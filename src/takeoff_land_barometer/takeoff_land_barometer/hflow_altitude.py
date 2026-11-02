#!/usr/bin/env python3
"""
hflow_altitude.py — NEW (2026-10-19, HFLOWALT)
==================================================================
ALTITUDE FROM THE HOLYBRO H-FLOW for the OFFBOARD flight programs.

WHY THIS EXISTS
    Since 2026-10-19 PX4 uses the H-Flow range as its height reference
    (EKF2_HGT_REF 2, EKF2_RNG_CTRL 1), but the EKF z is still not "the
    H-Flow": the range observes (terrain - z), and the EKF also fuses the
    GPS vertical velocity (EKF2_GPS_CTRL 5, 3D velocity) and the barometer.
    In ULog 191 (2026-10-19 11:53) the GPS reported ~0 m/s during a 0.3 m/s
    climb; the EKF z under-read the climb by 0.35-0.42 m and booked the
    difference into its terrain state. Code that flies on z - ground_z flew
    ~0.4 m higher than it believed. (PX4 v1.17 source: with RNG_CTRL 2 the
    height reference falls back to UNKNOWN after the start, so a parameter
    change alone does not pin z to the range either.)

    This module measures the altitude above the ground directly from the
    H-Flow. In "hold" mode BaroHoldMixin closes the altitude loop on it the
    same way baro_altitude.py did with the barometer:

        z_setpoint = z_ekf + gain * (alt_target - alt_hflow)

    PX4 still receives ordinary EKF-frame position setpoints. No PX4
    parameter is written; the pilot's manual modes are unchanged.

SIGNALS
    /mavros/distance_sensor/hflow  sensor_msgs/Range, 10 Hz
                                   (enabled by config/hflow_mavros.yaml)
    /mavros/imu/data               orientation -> tilt for the cos correction
    height of the IMU above the ground:  h = (range + lever_z) * cos(tilt)
        lever_z = 0.195 m: lens below the IMU (EKF2_RNG_POS_Z). On the
        ground the H-Flow reads ~0.13-0.15 m -> h ~0.33 m.
    altitude above the take-off point:   alt = h - h_ground

ESTIMATE MODES
    HFLOW     a valid sample within stale_s: alt from the H-Flow.
    EKF-HOLD  no valid sample for stale_s: EKF z + the last H-Flow offset
              (continuous, no step). When the H-Flow comes back, the step
              between the two is faded out over blend_s. The mixin aborts
              the mission after hflow_stale_abort_s in this mode.
    NO-REF    no ground reference yet.

LIMITS
    - Terrain following: the H-Flow measures the distance to whatever is
      below. Over a box, a table or a person the altitude drops and hold
      mode climbs by the same amount.
    - Samples outside [min_range, max_range] of the message, or taken at a
      tilt above max_tilt_deg, are dropped (count as missing).
    - A single sample more than glitch_m away from the filtered value is
      dropped; a jump that persists 3 samples is accepted (a real edge).
"""
import collections
import csv
import math
import os
import threading
import time

import numpy as np


class HFlowAltitudeEstimator:
    """H-Flow altitude above the take-off point + EKF-frame conversion.

    Thread-safe: fed from ROS callbacks, read from the setpoint timer and
    from the mission thread.
    """

    MODE_HFLOW    = "HFLOW"
    MODE_EKF_HOLD = "EKF-HOLD"
    MODE_NONE     = "NO-REF"

    def __init__(self, get_ekf_z, *, gain=1.0, lever_z=0.195, tau_s=0.2,
                 glitch_m=0.5, stale_s=0.5, max_tilt_deg=35.0,
                 max_cmd_offset=1.5, blend_s=1.0, imu_stale_s=0.5):
        self._get_ekf_z  = get_ekf_z
        self.gain        = float(gain)
        self.lever_z     = float(lever_z)
        self.tau         = max(0.02, float(tau_s))
        self.glitch_m    = float(glitch_m)
        self.stale_s     = float(stale_s)
        self.max_tilt    = math.radians(float(max_tilt_deg))
        self.max_cmd_off = float(max_cmd_offset)
        self.blend_s     = max(0.0, float(blend_s))
        self.imu_stale_s = float(imu_stale_s)

        self._lock = threading.Lock()
        # attitude
        self._tilt   = None      # rad, angle between body z and the vertical
        self._t_imu  = None
        # raw feed
        self._r_raw  = None      # last raw range (m), accepted or not
        self._h_raw  = None      # last accepted IMU height (m)
        self._h_filt = None      # low-passed
        self._t_last = None      # wall time of the last accepted sample
        self._n_msgs = 0
        self._n_drop = 0         # out of range / too tilted / not finite
        self._n_glitch = 0
        self._glitch_run = 0
        self._n_no_imu = 0       # samples corrected with tilt 0 (no IMU yet)
        self._hist = collections.deque(maxlen=1200)   # (t, h_raw, h_filt), 2 min @10 Hz
        # ground reference
        self.ready      = False
        self.h_ground   = None
        self.z_ground   = None
        # estimate state
        self.mode        = self.MODE_NONE
        self.off_locked  = None   # z_ekf - h_filt, refreshed while HFLOW
        self._blend_t0   = None
        self._blend_step = 0.0
        self.n_fallback  = 0      # HFLOW -> EKF-HOLD transitions
        self.last_alt    = 0.0

    # ── raw feed ──────────────────────────────────────────────────────────
    def cb_imu(self, msg):
        q = msg.orientation
        c = 1.0 - 2.0 * (float(q.x) ** 2 + float(q.y) ** 2)   # body z . world z
        if not math.isfinite(c):
            return
        with self._lock:
            self._tilt = math.acos(max(-1.0, min(1.0, c)))
            self._t_imu = time.time()

    def cb_range(self, msg):
        r = float(msg.range)
        now = time.time()
        with self._lock:
            self._n_msgs += 1
            self._r_raw = r
            lo, hi = float(msg.min_range), float(msg.max_range)
            if not math.isfinite(r) or (hi > lo > 0.0 and not (lo <= r <= hi)):
                self._n_drop += 1
                return
            if self._tilt is not None and self._t_imu is not None \
                    and now - self._t_imu <= self.imu_stale_s:
                tilt = self._tilt
            else:
                tilt = 0.0
                self._n_no_imu += 1
            if tilt > self.max_tilt:
                self._n_drop += 1
                return
            h = (r + self.lever_z) * math.cos(tilt)
            if self._h_filt is not None and self._t_last is not None:
                dt = now - self._t_last
                if abs(h - self._h_filt) > self.glitch_m and dt < self.stale_s:
                    self._glitch_run += 1
                    if self._glitch_run < 3:
                        self._n_glitch += 1
                        return
                    # persisted: a real step, restart the filter on it
                    self._h_filt = h
                else:
                    a = 1.0 - math.exp(-max(0.0, dt) / self.tau)
                    self._h_filt += a * (h - self._h_filt)
                self._glitch_run = 0
            else:
                self._h_filt = h
            self._h_raw  = h
            self._t_last = now
            self._hist.append((now, h, self._h_filt))

    # ── feed health ───────────────────────────────────────────────────────
    def stale_for(self):
        """Seconds since the last accepted sample (inf if none)."""
        with self._lock:
            if self._t_last is None:
                return float("inf")
            return time.time() - self._t_last

    def rate_hz(self, window_s=5.0):
        with self._lock:
            if len(self._hist) < 2:
                return 0.0
            now = time.time()
            n = sum(1 for t, _, _ in self._hist if now - t <= window_s)
            return n / window_s

    def window_stats(self, window_s):
        """Stats of the FILTERED IMU height over the last window_s.
        Returns dict(n, mean, std, spread, raw_std) or None if too few."""
        with self._lock:
            if not self._hist:
                return None
            now = time.time()
            f = [flt for t, _, flt in self._hist if now - t <= window_s]
            r = [raw for t, raw, _ in self._hist if now - t <= window_s]
        if len(f) < 3:
            return None
        f = np.asarray(f); r = np.asarray(r)
        return {"n": int(len(f)), "mean": float(f.mean()), "std": float(f.std()),
                "spread": float(f.max() - f.min()), "raw_std": float(r.std())}

    def snapshot(self):
        """Raw values for logging: (range, tilt_deg, h_filt)."""
        with self._lock:
            tilt = None if self._tilt is None else math.degrees(self._tilt)
            return self._r_raw, tilt, self._h_filt

    def counters(self):
        with self._lock:
            return {"msgs": self._n_msgs, "drop": self._n_drop,
                    "glitch": self._n_glitch, "no_imu": self._n_no_imu,
                    "fallback": self.n_fallback}

    # ── ground reference ──────────────────────────────────────────────────
    def set_ground(self, h_ground, z_ground):
        with self._lock:
            self.h_ground   = float(h_ground)
            self.z_ground   = float(z_ground)
            self.off_locked = float(z_ground) - float(h_ground)
            self.mode       = self.MODE_HFLOW
            self._blend_t0  = None
            self.ready      = True

    # ── estimate ──────────────────────────────────────────────────────────
    def altitude(self):
        """Altitude above the take-off point (m) and the mode used."""
        z_e = float(self._get_ekf_z())
        now = time.time()
        with self._lock:
            if not self.ready or self._h_filt is None:
                self.last_alt = 0.0
                return 0.0, self.MODE_NONE
            alt_hold = z_e - self.off_locked - self.h_ground
            fresh = self._t_last is not None and (now - self._t_last) <= self.stale_s
            if not fresh:
                if self.mode != self.MODE_EKF_HOLD:
                    self.mode = self.MODE_EKF_HOLD
                    self.n_fallback += 1
                    self._blend_t0 = None
                alt = alt_hold
            else:
                alt_h = self._h_filt - self.h_ground
                if self.mode == self.MODE_EKF_HOLD:
                    # back from a gap: fade out the step accumulated meanwhile
                    self._blend_t0, self._blend_step = now, alt_h - alt_hold
                self.mode = self.MODE_HFLOW
                self.off_locked = z_e - self._h_filt
                alt = alt_h
                if self._blend_t0 is not None and self.blend_s > 0.0:
                    w = min(1.0, (now - self._blend_t0) / self.blend_s)
                    alt = alt_h - (1.0 - w) * self._blend_step
                    if w >= 1.0:
                        self._blend_t0 = None
            self.last_alt = float(alt)
            return float(alt), self.mode

    def z_setpoint(self, alt_target):
        """EKF-frame z to command so that PX4's position error equals
        gain * (alt_target - altitude). Bounded by max_cmd_offset."""
        alt, _ = self.altitude()
        z_e = float(self._get_ekf_z())
        corr = self.gain * (float(alt_target) - alt)
        corr = max(-self.max_cmd_off, min(self.max_cmd_off, corr))
        return z_e + corr


class HFlowCsvLog:
    """10 Hz CSV next to the flight (the ULog keeps distance_sensor at 1 Hz).
    Never raises: on any I/O error it disables itself."""

    COLUMNS = ("t", "phase", "armed", "ekf_z", "ekf_vz", "ekf_alt",
               "hflow_range", "tilt_deg", "hflow_h", "hflow_alt", "hflow_mode",
               "hflow_age", "baro_alt", "alt_used", "sp_nominal", "sp_sent")

    def __init__(self, directory, node_name):
        self.path = None
        self._f = None
        self._w = None
        self._n = 0
        try:
            d = os.path.expanduser(directory)
            os.makedirs(d, exist_ok=True)
            self.path = os.path.join(
                d, f"hflow_{time.strftime('%Y%m%d-%H%M%S')}_{node_name}.csv")
            self._f = open(self.path, "w", newline="")
            self._w = csv.writer(self._f)
            self._w.writerow(self.COLUMNS)
        except Exception:
            self.close()

    @property
    def ok(self):
        return self._w is not None

    def row(self, values):
        if self._w is None:
            return
        try:
            self._w.writerow([("" if v is None else
                               (f"{v:.4f}" if isinstance(v, float) else v))
                              for v in values])
            self._n += 1
            if self._n % 10 == 0:
                self._f.flush()
        except Exception:
            self.close()

    def close(self):
        try:
            if self._f is not None:
                self._f.close()
        except Exception:
            pass
        self._f = None
        self._w = None
