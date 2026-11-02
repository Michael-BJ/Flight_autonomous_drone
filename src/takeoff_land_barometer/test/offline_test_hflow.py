#!/usr/bin/env python3
"""NEW (2026-10-19, HFLOWALT): offline tests for HFlowAltitudeEstimator
(no ROS spin, deterministic clock).

Run with the system python under the ROS env:
    /usr/bin/python3 offline_test_hflow.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/drone_ws/src/takeoff_land_barometer"))
import takeoff_land_barometer.hflow_altitude as ha  # noqa: E402
from takeoff_land_barometer.hflow_altitude import HFlowAltitudeEstimator  # noqa: E402
from sensor_msgs.msg import Imu, Range  # noqa: E402

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(("PASS " if cond else "FAIL ") + name + ("  " + detail if detail else ""))


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


clock = Clock()
ha.time.time = clock
ekf = {"z": 0.0}
LEVER = 0.195
GROUND_RANGE = 0.135          # H-Flow reading on the ground (10/19 logs: 0.13-0.15)


def make():
    return HFlowAltitudeEstimator(lambda: ekf["z"], gain=1.0, lever_z=LEVER,
                                  tau_s=0.2, glitch_m=0.5, stale_s=0.5,
                                  max_tilt_deg=35.0, max_cmd_offset=1.5, blend_s=1.0)


def imu(est, tilt_deg):
    m = Imu()
    half = math.radians(tilt_deg) / 2.0       # pure roll
    m.orientation.w, m.orientation.x = math.cos(half), math.sin(half)
    est.cb_imu(m)


def rng(est, r, dt=0.1, lo=0.08, hi=30.0):
    m = Range(); m.range = float(r); m.min_range = lo; m.max_range = hi
    clock.t += dt
    est.cb_range(m)


noise = np.random.default_rng(3)

# ── 1. ground reference ────────────────────────────────────────────────────
est = make()
imu(est, 0.0)
ekf["z"] = -0.97                              # 10/19 morning: EKF z ~ -1 m on the ground
for _ in range(60):
    imu(est, 0.0)
    rng(est, GROUND_RANGE + noise.normal(0, 0.005))
st = est.window_stats(5.0)
check("ground: filtered std tiny", st["std"] < 0.01, f"std={st['std']:.4f}")
check("ground: h = range + lever ~0.33", abs(st["mean"] - (GROUND_RANGE + LEVER)) < 0.01,
      f"h={st['mean']:.3f}")
est.set_ground(st["mean"], ekf["z"])
alt, mode = est.altitude()
check("ground: alt ~0, HFLOW", abs(alt) < 0.02 and mode == "HFLOW", f"alt={alt:.3f} {mode}")

# ── 2. climb to 1.15 m while the EKF under-reads by 35 % (log 191) ─────────
for i in range(80):
    true_alt = min(1.15, i * 0.03)
    ekf["z"] = -0.97 + 0.65 * true_alt        # EKF climbs only 0.75 m of 1.15
    imu(est, 3.0)
    rng(est, (GROUND_RANGE + true_alt + LEVER) / math.cos(math.radians(3.0)) - LEVER
        + noise.normal(0, 0.005))
alt, mode = est.altitude()
check("log-191 case: H-Flow alt follows the truth, not the EKF",
      abs(alt - 1.15) < 0.03, f"alt={alt:.3f} ekf_alt={ekf['z'] + 0.97:.3f}")
zsp = est.z_setpoint(0.75)
check("z_setpoint pulls down by the H-Flow error (0.75 - 1.15)",
      abs(zsp - (ekf["z"] + (0.75 - alt))) < 1e-6 and zsp < ekf["z"],
      f"zsp={zsp:.3f} z_ekf={ekf['z']:.3f}")

# ── 3. tilt correction ─────────────────────────────────────────────────────
imu(est, 20.0)
for _ in range(20):
    imu(est, 20.0)
    rng(est, (GROUND_RANGE + 1.15 + LEVER) / math.cos(math.radians(20.0)) - LEVER)
alt, _ = est.altitude()
check("20 deg tilt: slant range corrected", abs(alt - 1.15) < 0.02, f"alt={alt:.3f}")
n0 = est.counters()["drop"]
imu(est, 40.0)
rng(est, 3.0)
check("40 deg tilt: sample dropped", est.counters()["drop"] == n0 + 1)
imu(est, 0.0)

# ── 4. single glitch rejected, persistent step accepted ────────────────────
for _ in range(10):
    rng(est, GROUND_RANGE + 1.15)
a0, _ = est.altitude()
rng(est, 0.2)                                 # one bad sample (bird, dust)
a1, _ = est.altitude()
check("single 1 m glitch rejected", abs(a1 - a0) < 0.01, f"{a0:.3f} -> {a1:.3f}")
for _ in range(5):                            # flying over a 0.6 m box
    rng(est, GROUND_RANGE + 1.15 - 0.6)
a2, _ = est.altitude()
check("persistent 0.6 m step accepted after 3 samples (terrain following)",
      abs(a2 - 0.55) < 0.05, f"alt={a2:.3f}")
for _ in range(10):
    rng(est, GROUND_RANGE + 1.15)
check("off the box: back to 1.15", abs(est.altitude()[0] - 1.15) < 0.05)

# ── 5. out-of-range samples dropped ────────────────────────────────────────
n0 = est.counters()["drop"]
rng(est, 0.05)                                # below min_range 0.08
rng(est, float("nan"))
check("below min_range / NaN dropped", est.counters()["drop"] == n0 + 2)

# ── 6. gap -> EKF-HOLD without a step, then blend back ─────────────────────
for _ in range(10):
    rng(est, GROUND_RANGE + 1.0)
a_before, m_before = est.altitude()
z_before = ekf["z"]
clock.t += 0.6                                # no sample for 0.6 s > stale_s
a_gap, m_gap = est.altitude()
check("gap: EKF-HOLD, continuous", m_gap == "EKF-HOLD" and abs(a_gap - a_before) < 0.02,
      f"{a_before:.3f}/{m_before} -> {a_gap:.3f}/{m_gap}")
ekf["z"] = z_before + 0.10                    # EKF says +0.10 during the gap
clock.t += 1.0
a_gap2, _ = est.altitude()
check("gap: dead-reckons on the EKF", abs(a_gap2 - (a_before + 0.10)) < 0.02, f"{a_gap2:.3f}")
check("stale_for counts the gap", est.stale_for() > 1.5, f"{est.stale_for():.2f}s")
rng(est, GROUND_RANGE + 1.0)                  # back: truth still 1.0
a_back, m_back = est.altitude()
check("back: HFLOW, step faded (no jump)", m_back == "HFLOW" and abs(a_back - a_gap2) < 0.03,
      f"{a_gap2:.3f} -> {a_back:.3f}")
for _ in range(15):
    rng(est, GROUND_RANGE + 1.0)
check("back: after blend_s the estimate is the H-Flow", abs(est.altitude()[0] - 1.0) < 0.02,
      f"{est.altitude()[0]:.3f}")
check("fallback counted", est.counters()["fallback"] == 1)

# ── 7. no IMU: tilt assumed 0, counted ─────────────────────────────────────
est2 = make()
rng(est2, 0.5)
check("no IMU: sample used with tilt 0 and counted", est2.counters()["no_imu"] == 1
      and est2.snapshot()[2] is not None)

# ── 8. z_setpoint bounded ──────────────────────────────────────────────────
zsp = est.z_setpoint(5.0)
check("z_setpoint bounded by max_cmd_offset", abs(zsp - ekf["z"] - 1.5) < 1e-6,
      f"{zsp - ekf['z']:.3f}")

n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} passed")
sys.exit(1 if n_fail else 0)
