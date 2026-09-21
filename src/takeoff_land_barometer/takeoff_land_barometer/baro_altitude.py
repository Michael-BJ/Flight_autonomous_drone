#!/usr/bin/env python3
"""
baro_altitude.py
==================================================================
BAROMETRIC ALTITUDE HOLD for the OFFBOARD flight programs (2026-09-13).

WHY THIS EXISTS
    The EKF height on this drone follows the GPS (EKF2_HGT_REF = 1). On
    2026-09-13 the GPS height wandered 1.2-3.6 m peak-to-peak while the
    drone sat on the ground, and during the 15:05 fm_deploy flight the
    drone lost ~0.9 m of physical altitude in 40 s while the EKF insisted it
    was still at the target (PX4 holds the EKF z, so the drone physically
    follows every GPS wander). Nothing on the companion computer can fix
    the estimator itself, but the programs can close their own altitude
    loop on the barometer instead of trusting the EKF z:

        z_setpoint = z_ekf + gain * (alt_target - alt_baro)

    PX4 still receives an ordinary position setpoint in its own EKF frame
    (nothing about the PX4 side changes: no parameter is written, the
    pilot's manual modes behave exactly as before). The setpoint is just
    re-computed every tick so that the POSITION ERROR PX4 sees equals the
    BAROMETRIC altitude error. A slow GPS wander or a sudden GPS jump in
    z_ekf shifts the setpoint by the same amount and PX4 sees no error, so
    the drone does not move; the barometer decides the altitude.

    X and Y are untouched: they stay GPS / EKF, exactly as before.

WHAT IT CAN AND CANNOT DO
    - Cancels slow GPS-height drift and GPS z jumps in the POSITION loop.
    - It cannot remove GPS noise from PX4's own VERTICAL VELOCITY estimate
      (the EKF fuses GPS velocity). Fast EKF-z jitter still makes PX4 bob a
      little; the pre-flight gate below still refuses a wildly jittering
      EKF for that reason.
    - Barometers drift with weather (slowly), with temperature (Pixhawk
      warm-up: about -3.5 m in the first 2 minutes after boot on this
      board, measured 2026-09-13) and read LOW under propeller downwash
      close to the ground (ground effect). The estimator therefore:
        * uses the barometer relative to the value measured ON THE GROUND
          right before arming (weather drift over a few minutes is small,
          but let the Pixhawk warm up before flying);
        * inside `ground_effect_alt` (default 1.0 m) does NOT use the
          barometer at all: it dead-reckons with the EKF z from the last
          trusted barometric fix ("EKF-LOCK" mode), i.e. the first metre of
          the climb and the last metre of the descent behave exactly like
          the existing takeoff_land code.
    - Measured on this drone (2026-09-13 22:4x, indoors): /mavros/altitude
      arrives at 10 Hz, the raw barometric altitude has ~0.085 m RMS white
      noise. A 1 s low-pass leaves ~3 cm, which the loop below tolerates.

SIGNALS
    /mavros/altitude .monotonic  = PX4 vehicle_air_data.baro_alt_meter,
                                   the pure barometric altitude (NOT the
                                   EKF). .amsl/.local/.relative are EKF.
    EKF z                        = /px4/sensors local_z (the same value the
                                   existing nodes fly on).

This module is deliberately free of any mission logic. BaroHoldMixin
plugs it into TakeoffLandNode / ForwardMoveNode / FMInferenceRealNode
by (1) wrapping the setpoint publisher so every PositionTarget gets its z
re-computed, (2) replacing the pre-flight ground-reference wait, and
(3) providing altitude/sanity helpers. The mission sequences themselves
are inherited unchanged from the original packages.
"""
import collections
import math
import threading
import time

import numpy as np
from mavros_msgs.msg import Altitude, PositionTarget
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

_ALT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=10,
    durability=DurabilityPolicy.VOLATILE)


class BaroAltitudeEstimator:
    """Barometric altitude above the take-off point + EKF-frame conversion.

    Thread-safe: fed from a ROS callback, read from the setpoint timer and
    from the mission thread.
    """

    MODE_EKF_LOCK = "EKF-LOCK"   # near the ground: EKF z, offset locked
    MODE_BARO     = "BARO"       # in the air: barometer
    MODE_STALE    = "STALE"      # barometer stream stopped: EKF, offset locked
    MODE_EKF      = "EKF"        # NEW (2026-09-15, GAIN0): gain 0, EKF altitude only

    def __init__(self, node, get_ekf_z, *, gain=0.7, tau_s=1.0,
                 ground_effect_alt=1.0, blend_s=2.0, glitch_m=1.5,
                 stale_s=1.0, max_cmd_offset=1.5, topic="/mavros/altitude"):
        self._node       = node
        self._get_ekf_z  = get_ekf_z
        self.gain        = float(gain)
        self.tau         = max(0.05, float(tau_s))
        self.ge_alt      = float(ground_effect_alt)
        self.blend_s     = max(0.0, float(blend_s))
        self.glitch_m    = float(glitch_m)
        self.stale_s     = float(stale_s)
        self.max_cmd_off = float(max_cmd_offset)

        self._lock = threading.Lock()
        # raw feed
        self._h_raw    = None    # last accepted raw barometric altitude (m)
        self._h_filt   = None    # low-passed
        self._t_last   = None    # wall time of the last accepted sample
        self._n_msgs   = 0
        self._n_glitch = 0
        self._glitch_run = 0
        self._hist     = collections.deque(maxlen=1200)   # (t, raw, filt), 2 min @10 Hz
        # ground reference
        self.ready       = False
        self.h_ground    = None    # barometric altitude on the ground (filtered)
        self.z_ground    = None    # EKF z on the ground (same window)
        # mode / offset
        self.mode        = self.MODE_EKF_LOCK
        self.off_locked  = None    # z_ekf - h_filt captured when trusted
        self._blend_t0   = None
        self._blend_step = 0.0     # (baro alt - EKF-LOCK alt) at the switch, faded out
        self._mode_t     = 0.0     # when the current mode was entered
        self.min_dwell_s = 1.0     # no mode switch sooner than this after the last
        self.last_alt    = 0.0
        self.last_detail = ""
        self.n_mode_switch = 0

    # ── raw feed ──────────────────────────────────────────────────────────
    def cb_altitude(self, msg: Altitude):
        h = float(msg.monotonic)
        if not math.isfinite(h):
            return
        now = time.time()
        with self._lock:
            self._n_msgs += 1
            if self._h_filt is not None and self._t_last is not None:
                dt = now - self._t_last
                # Glitch rejection: a single sample far from the filtered
                # value is dropped; if it persists (a real step) it is
                # accepted after 3 samples so the filter can follow it.
                if abs(h - self._h_filt) > self.glitch_m and dt < self.stale_s:
                    self._glitch_run += 1
                    if self._glitch_run < 3:
                        self._n_glitch += 1
                        return
                self._glitch_run = 0
                a = 1.0 - math.exp(-max(0.0, dt) / self.tau)
                self._h_filt += a * (h - self._h_filt)
            else:
                self._h_filt = h
            self._h_raw  = h
            self._t_last = now
            self._hist.append((now, h, self._h_filt))

    def stale_for(self):
        """Seconds since the last accepted barometer sample (inf if none)."""
        with self._lock:
            if self._t_last is None:
                return float("inf")
            return time.time() - self._t_last

    def rate_hz(self, window_s=5.0):
        with self._lock:
            if len(self._hist) < 2:
                return 0.0
            now = self._hist[-1][0]
            n = sum(1 for t, _, _ in self._hist if now - t <= window_s)
            return n / window_s

    def window_stats(self, window_s):
        """Stats of the FILTERED barometric altitude over the last window_s.
        Returns dict(n, mean, std, spread, raw_std) or None if too few."""
        with self._lock:
            if not self._hist:
                return None
            now = self._hist[-1][0]
            f = [flt for t, _, flt in self._hist if now - t <= window_s]
            r = [raw for t, raw, _ in self._hist if now - t <= window_s]
        if len(f) < 3:
            return None
        f = np.asarray(f); r = np.asarray(r)
        return {"n": int(len(f)), "mean": float(f.mean()), "std": float(f.std()),
                "spread": float(f.max() - f.min()), "raw_std": float(r.std())}

    # ── ground reference ──────────────────────────────────────────────────
    def set_ground(self, h_ground, z_ground):
        with self._lock:
            self.h_ground   = float(h_ground)
            self.z_ground   = float(z_ground)
            self.off_locked = float(z_ground) - float(h_ground)
            self.mode       = self.MODE_EKF_LOCK
            self._mode_t    = time.time()
            self._blend_t0  = None
            self.ready      = True

    # ── estimate ──────────────────────────────────────────────────────────
    def altitude(self):
        """Current altitude above the take-off point (m) and the mode used.

        Mode logic (hysteresis on the CURRENT estimate, >= min_dwell_s
        between switches, see the module docstring):
          EKF-LOCK -> BARO  when the estimate climbs above
                            ground_effect_alt + 0.2 m (the step between the
                            two estimates is faded out over blend_s)
          BARO -> EKF-LOCK  when the estimate drops below
                            ground_effect_alt - 0.1 m (the offset is kept
                            fresh in BARO, so the estimate is continuous)
          any -> STALE      when no barometer sample for stale_s (EKF
                            dead-reckoning from the last good offset)
        """
        z_e = float(self._get_ekf_z())
        now = time.time()
        with self._lock:
            if not self.ready or self._h_filt is None:
                self.last_alt, self.last_detail = 0.0, "no reference"
                return 0.0, self.mode
            # NEW (2026-09-15, GAIN0): gain <= 0 = no barometric loop at all.
            # PX4's EKF already flies on the barometer (EKF2_HGT_REF 0,
            # EKF2_GPS_CTRL 5, read back 2026-09-15 00:30); a second loop on
            # the raw barometer made the drone bob. Altitude = EKF z above
            # the ground reference, like takeoff_land.
            if self.gain <= 0.0:
                alt = z_e - self.z_ground
                self.mode = self.MODE_EKF
                self.last_alt = float(alt)
                self.last_detail = (f"ekf={alt:+.2f} baro={self._h_filt - self.h_ground:+.2f} "
                                    "(gain 0: baro not used)")
                return float(alt), self.mode
            h = self._h_filt
            stale = (now - self._t_last) > self.stale_s
            off_now = z_e - h
            alt_b = h - self.h_ground
            alt_l = z_e - self.off_locked - self.h_ground

            dwell_ok = (now - self._mode_t) >= self.min_dwell_s
            if stale:
                if self.mode != self.MODE_STALE:
                    self.mode = self.MODE_STALE
                    self._mode_t = now
                    self.n_mode_switch += 1
                    self._blend_t0 = None
                alt = alt_l
            else:
                if self.mode == self.MODE_STALE:
                    # stream is back: continue with the locked offset; the
                    # hysteresis below decides where to go next
                    self.mode = self.MODE_EKF_LOCK
                    self._mode_t = now
                if self.mode == self.MODE_EKF_LOCK:
                    alt = alt_l
                    if alt > self.ge_alt + 0.2 and dwell_ok:
                        self.mode = self.MODE_BARO
                        self._mode_t = now
                        self.n_mode_switch += 1
                        # fade out the STEP between the two estimates (the
                        # dynamics are the barometer's from now on)
                        self._blend_t0, self._blend_step = now, alt_b - alt_l
                        alt = alt_b - self._blend_step
                elif self.mode == self.MODE_BARO:
                    # Keep the locked offset fresh while the barometer is
                    # trusted, so a re-lock near the ground or a stale spell
                    # continues WITHOUT a step (z_ekf and the filtered
                    # barometer are both smooth; a GPS jump is captured at
                    # once).
                    self.off_locked = off_now
                    alt = alt_b
                    if self._blend_t0 is not None and self.blend_s > 0.0:
                        w = min(1.0, (now - self._blend_t0) / self.blend_s)
                        alt = alt_b - (1.0 - w) * self._blend_step
                        if w >= 1.0:
                            self._blend_t0 = None
                    if alt < self.ge_alt - 0.1 and dwell_ok:
                        self.mode = self.MODE_EKF_LOCK
                        self._mode_t = now
                        self.n_mode_switch += 1
                        self._blend_t0 = None
                        alt = z_e - self.off_locked - self.h_ground
            self.last_alt = float(alt)
            self.last_detail = (f"baro={alt_b:+.2f} ekf={alt_l:+.2f} "
                                f"off={off_now:+.2f}")
            return float(alt), self.mode

    def z_setpoint(self, alt_target):
        """EKF-frame z to command so that PX4's position error equals
        gain * (alt_target - altitude). Bounded by max_cmd_offset."""
        if self.gain <= 0.0:
            # NEW (2026-09-15, GAIN0): NOT z_ekf + 0 (that would command
            # "stay where you are" and never reach the target) — the plain
            # nominal setpoint, exactly what takeoff_land sends.
            self.altitude()   # keeps last_alt / mode fresh for the status line
            return float(self.z_ground) + float(alt_target)
        alt, _ = self.altitude()
        z_e = float(self._get_ekf_z())
        corr = self.gain * (float(alt_target) - alt)
        corr = max(-self.max_cmd_off, min(self.max_cmd_off, corr))
        return z_e + corr

    def stats_line(self):
        with self._lock:
            n, g = self._n_msgs, self._n_glitch
        return (f"balt={self.last_alt:5.2f}m {self.mode} "
                f"[{self.last_detail}] baro {self.rate_hz():.0f}Hz "
                f"msgs={n} glitch={g}")


class BaroZPublisher:
    """Wraps the node's PositionTarget publisher: every message that carries
    a position z (IGNORE_PZ not set) gets its z converted from the NOMINAL
    EKF-frame value the mission code computed (ground_z + altitude) into the
    barometer-corrected setpoint. Messages published before the ground
    reference exists pass through unchanged. Nothing else is touched."""

    def __init__(self, real_pub, node):
        self._real = real_pub
        self._node = node
        self.n_rewritten = 0
        self.last_nominal = None
        self.last_sent = None

    def publish(self, msg):
        try:
            est = self._node._baro
            if (isinstance(msg, PositionTarget) and est.ready
                    and est.gain > 0.0   # NEW (2026-09-15, GAIN0): gain 0 -> pass through
                    and not (msg.type_mask & PositionTarget.IGNORE_PZ)):
                nominal = float(msg.position.z)
                alt_target = nominal - float(self._node._ground_z)
                msg.position.z = float(est.z_setpoint(alt_target))
                self.n_rewritten += 1
                self.last_nominal, self.last_sent = nominal, msg.position.z
        except Exception:
            # never let a bookkeeping error stop the setpoint stream
            pass
        return self._real.publish(msg)

    def __getattr__(self, name):
        return getattr(self._real, name)


class BaroHoldMixin:
    """Mixin for the flight nodes. Put it FIRST in the base list so its
    methods override the originals:

        class TakeoffLandBaroNode(BaroHoldMixin, TakeoffLandNode): ...

    Call self._baro_init() at the end of __init__ (after the original
    constructor created the publisher and read its parameters).
    """

    # ── setup ──────────────────────────────────────────────────────────────
    def _baro_init(self, log_tag="[BARO]"):
        self._baro_tag = log_tag
        p = self.declare_parameter
        # NEW (2026-09-15, GAIN0): default 0.7 -> 0.0 (0 = no barometric loop,
        # EKF altitude, nominal setpoints). 0.7 = the 2026-09-13 loop.
        p("baro_gain",              0.0)    # loop gain on the barometric error
        p("baro_tau_s",             1.0)    # low-pass on the raw barometer
        p("baro_ground_effect_alt", 1.0)    # m; below this: EKF dead-reckoning
        p("baro_blend_s",           2.0)    # s; blend EKF-LOCK -> BARO
        p("baro_glitch_m",          1.5)    # m; single-sample outlier reject
        p("baro_stale_s",           1.0)    # s without samples -> EKF fallback
        p("baro_stale_abort_s",     5.0)    # s without samples -> abort (0=never)
        p("baro_max_cmd_offset",    1.5)    # m; |z_sp - z_ekf| bound
        # pre-flight gate on the barometer (filtered value, ekf_window_s)
        p("baro_gate_std",          0.08)   # m
        p("baro_gate_drift",        0.30)   # m peak-to-peak
        p("baro_min_rate_hz",       5.0)    # Hz
        # pre-flight gate on the EKF z (jitter only — slow drift is what the
        # barometer loop compensates, |z| is irrelevant to it)
        p("ekf_gate_std",           0.15)   # m (takeoff_land uses 0.08)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._baro = BaroAltitudeEstimator(
            self, self._baro_ekf_z,
            gain=float(g("baro_gain")), tau_s=float(g("baro_tau_s")),
            ground_effect_alt=float(g("baro_ground_effect_alt")),
            blend_s=float(g("baro_blend_s")), glitch_m=float(g("baro_glitch_m")),
            stale_s=float(g("baro_stale_s")),
            max_cmd_offset=float(g("baro_max_cmd_offset")))
        self._baro_stale_abort_s = float(g("baro_stale_abort_s"))
        self._baro_gate_std      = float(g("baro_gate_std"))
        self._baro_gate_drift    = float(g("baro_gate_drift"))
        self._baro_min_rate      = float(g("baro_min_rate_hz"))
        self._ekf_gate_std       = float(g("ekf_gate_std"))
        self._baro_stale_latched = False

        # The host node declared max_ground_z / max_ground_drift with
        # takeoff_land's defaults (1.0 m / 0.2 m). For the barometric loop
        # the absolute EKF z is irrelevant and slow drift is compensated, so
        # unless the launch file / command line set them explicitly, use the
        # barometer defaults (the launch files pass the same values).
        overrides = getattr(self, "_parameter_overrides", {}) or {}
        if "max_ground_z" not in overrides and hasattr(self, "_max_ground_z"):
            self._max_ground_z = 1000.0
        if "max_ground_drift" not in overrides and hasattr(self, "_max_gnd_drift"):
            self._max_gnd_drift = 0.5

        self.create_subscription(Altitude, "/mavros/altitude",
                                 self._baro.cb_altitude, _ALT_QOS)
        # Every setpoint the inherited code publishes goes through here.
        self._pub_sp = BaroZPublisher(self._pub_sp, self)

        self.get_logger().info("=" * 62)
        if self._baro.gain <= 0.0:   # NEW (2026-09-15, GAIN0)
            self.get_logger().info(
                f"{log_tag} baro_gain=0: NO barometric loop. Setpoint z = target "
                "(like takeoff_land); altitude for takeoff / watchdog / status = "
                "PX4 EKF z (PX4 already fuses the barometer). The barometer "
                "stream is still checked (pre-flight gate, stale abort).")
        self.get_logger().info(
            f"{log_tag} BAROMETRIC ALTITUDE HOLD: z from /mavros/altitude "
            f"(monotonic), x/y stay GPS/EKF. gain={self._baro.gain:.2f} "
            f"tau={self._baro.tau:.1f}s ground-effect zone <"
            f"{self._baro.ge_alt:.1f}m (EKF dead-reckoning there)")
        self.get_logger().info(
            f"{log_tag} PX4 parameters are NOT touched (EKF2_HGT_REF stays as "
            "it is). Pilot modes behave exactly as before.")
        self.get_logger().info(
            f"{log_tag} ground gate: EKF |z|<={getattr(self, '_max_ground_z', float('nan')):.0f}m "
            f"spread<={getattr(self, '_max_gnd_drift', float('nan')):.2f}m "
            f"std<{self._ekf_gate_std:.2f}m | baro std<{self._baro_gate_std:.2f}m "
            f"spread<={self._baro_gate_drift:.2f}m rate>={self._baro_min_rate:.0f}Hz")
        self.get_logger().info("=" * 62)

    # ── accessors that differ between the node families ───────────────────
    def _baro_ekf_z(self):
        ds = getattr(self, "_drone_state", None)
        if ds is not None:
            return float(ds.global_pos[2])
        return float(self._pos[2])

    def _baro_ekf_vz(self):
        ds = getattr(self, "_drone_state", None)
        if ds is not None:
            return float(ds.global_vel[2])
        return float(self._vel[2])

    def _baro_cancelled(self):
        """True when an abort flag of the host node is set."""
        ab = getattr(self, "_aborted", None)
        if callable(ab) and ab():
            return True
        if getattr(self, "_abort_reason", None) is not None:
            return True
        return bool(getattr(self, "_kill_latched", False)
                    or getattr(self, "_disarm_abort", False)
                    or getattr(self, "_rc_override", False)
                    or getattr(self, "_link_lost", False))

    def _baro_alt(self):
        """Altitude above the take-off point (m) from the estimator."""
        return self._baro.altitude()[0]

    # ── pre-flight ground reference (replaces _wait_ekf_stable) ───────────
    def _wait_ekf_stable(self, tol=0.08, stable_dur=5.0, timeout=60.0,
                         pre_wait=None):
        """Determine the ground references, or None if not trustworthy.

        Same shape as the original (pre-wait, window, held stable_dur,
        timeout) but with the criteria the barometric loop actually needs:

          barometer (filtered):  rate >= baro_min_rate_hz, std < baro_gate_std,
                                 spread <= baro_gate_drift   — the reference
                                 everything is measured against must be quiet
          EKF z:                 std < ekf_gate_std (jitter). PX4 flies on
                                 its own vertical VELOCITY, which this loop
                                 cannot fix, so a violently jittering EKF is
                                 still refused. Slow drift (max_ground_drift)
                                 and |z| (max_ground_z) are still applied,
                                 with the relaxed defaults of the barometer
                                 launch files, because the loop compensates
                                 exactly those.
        Returns the EKF ground z (the inherited code stores it as
        self._ground_z and keeps computing NOMINAL setpoints from it) and
        stores the barometric ground level in the estimator.
        """
        tag = self._baro_tag
        if pre_wait is None:
            pre_wait = float(getattr(self, "_ekf_pre_wait", 10.0))
        self.get_logger().info(
            f"{tag} Waiting {pre_wait:.0f}s for GPS/EKF/baro initial convergence...")
        t_pre = time.time()
        while time.time() - t_pre < pre_wait:
            if self._baro_cancelled():
                return None
            time.sleep(0.1)

        win_s = float(getattr(self, "_ekf_window_s", 5.0))
        max_drift = float(getattr(self, "_max_gnd_drift", 0.5))
        max_gz = float(getattr(self, "_max_ground_z", 1000.0))
        n_win = max(10, int(win_s / 0.1))
        self.get_logger().info(
            f"{tag} Ground reference: baro(filtered) std<{self._baro_gate_std}m "
            f"spread<={self._baro_gate_drift}m rate>={self._baro_min_rate:.0f}Hz | "
            f"EKF z std<{self._ekf_gate_std}m spread<={max_drift}m |z|<={max_gz}m "
            f"| over {win_s:.1f}s, held {stable_dur:.0f}s (timeout {timeout:.0f}s)")

        t0 = time.time()
        z_hist = []
        stable_start = None
        last_report = 0.0
        z_ground = h_ground = None
        while time.time() - t0 < timeout:
            if self._baro_cancelled():
                self.get_logger().warn(f"{tag} ground reference cancelled (abort).")
                return None
            z_hist.append(self._baro_ekf_z())
            if len(z_hist) > n_win:
                z_hist.pop(0)
            problem = None
            bs = self._baro.window_stats(win_s)
            if len(z_hist) < n_win:
                problem = f"collecting samples ({len(z_hist)}/{n_win})"
            elif bs is None:
                problem = "no barometer data on /mavros/altitude yet"
            elif self._baro.rate_hz() < self._baro_min_rate:
                problem = (f"barometer rate {self._baro.rate_hz():.1f} Hz "
                           f"(need >= {self._baro_min_rate:.0f})")
            elif bs["std"] > self._baro_gate_std:
                problem = f"baro jitter std={bs['std']:.3f}m (need <{self._baro_gate_std}m)"
            elif bs["spread"] > self._baro_gate_drift:
                problem = (f"baro drifting {bs['spread']:.2f}m over {win_s:.0f}s "
                           f"(need <={self._baro_gate_drift}m)")
            else:
                win = np.asarray(z_hist[-n_win:])
                z_std, z_pp, z_mean = float(win.std()), float(win.max() - win.min()), float(win.mean())
                if z_std > self._ekf_gate_std:
                    problem = f"EKF z jitter std={z_std:.3f}m (need <{self._ekf_gate_std}m)"
                elif z_pp > max_drift:
                    problem = (f"EKF z drifting {z_pp:.2f}m over {win_s:.0f}s "
                               f"(need <={max_drift}m)")
                elif abs(z_mean) > max_gz:
                    problem = f"EKF z={z_mean:.2f}m on the ground (need |z|<={max_gz}m)"
            now = time.time()
            if problem is None:
                if stable_start is None:
                    stable_start = now
                elif now - stable_start >= stable_dur:
                    win = np.asarray(z_hist[-n_win:])
                    z_ground = float(win.mean())
                    h_ground = float(bs["mean"])
                    self._baro.set_ground(h_ground, z_ground)
                    self.get_logger().info(
                        f"{tag} Ground reference OK. baro h_ground={h_ground:.2f}m "
                        f"(std={bs['std']:.3f}, spread={bs['spread']:.2f}m, raw std "
                        f"{bs['raw_std']:.3f}) | EKF ground_z={z_ground:.3f}m "
                        f"(std={float(win.std()):.3f}, spread={float(win.max()-win.min()):.2f}m) "
                        f"| offset ekf-baro={z_ground - h_ground:+.2f}m")
                    return z_ground
            else:
                stable_start = None
                if now - last_report >= 5.0:
                    last_report = now
                    self.get_logger().warn(
                        f"{tag} reference not usable yet: {problem} "
                        f"({timeout - (now - t0):.0f}s left)")
            time.sleep(0.1)

        self.get_logger().error("=" * 62)
        self.get_logger().error(
            f"{tag} GROUND REFERENCE NEVER BECAME TRUSTWORTHY within the timeout.")
        bs = self._baro.window_stats(win_s)
        if bs:
            self.get_logger().error(
                f"{tag} baro last window: mean={bs['mean']:.2f} std={bs['std']:.3f} "
                f"spread={bs['spread']:.2f}m rate={self._baro.rate_hz():.1f}Hz")
        if z_hist:
            win = np.asarray(z_hist[-min(len(z_hist), n_win):])
            self.get_logger().error(
                f"{tag} EKF z last window: z={float(win.mean()):.2f} "
                f"std={float(win.std()):.3f} spread={float(win.max()-win.min()):.2f}m")
        self.get_logger().error(f"{tag} REFUSING TO FLY on an unverified reference.")
        self.get_logger().error("=" * 62)
        return None

    # ── barometer health during the mission ───────────────────────────────
    def _baro_health_problem(self):
        """None while the barometer is usable, else the abort reason."""
        if self._baro_stale_abort_s <= 0.0:
            return None
        s = self._baro.stale_for()
        if s > self._baro_stale_abort_s:
            return (f"BAROMETER STREAM LOST for {s:.1f}s "
                    f"(limit {self._baro_stale_abort_s:.0f}s)")
        return None
