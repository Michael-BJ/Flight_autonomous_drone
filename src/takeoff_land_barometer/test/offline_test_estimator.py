#!/usr/bin/env python3
"""Offline tests for BaroAltitudeEstimator / BaroZPublisher (no ROS spin).

Run with the system python under the ROS env:
    /usr/bin/python3 test_estimator.py
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/drone_ws/src/takeoff_land_barometer"))
from takeoff_land_barometer.baro_altitude import BaroAltitudeEstimator, BaroZPublisher  # noqa: E402
from mavros_msgs.msg import Altitude, PositionTarget  # noqa: E402

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(("PASS " if cond else "FAIL ") + name + ("  " + detail if detail else ""))


class Clock:
    """Monkey-patchable time.time for deterministic tests."""
    def __init__(self):
        self.t = 1000.0
    def __call__(self):
        return self.t


def make_est(clock, **kw):
    import takeoff_land_barometer.baro_altitude as ba
    ba.time.time = clock            # module-level patch
    holder = {"z": 0.0}
    est = BaroAltitudeEstimator(None, lambda: holder["z"], **kw)
    return est, holder


def feed(est, h, clock, dt=0.1):
    m = Altitude(); m.monotonic = float(h)
    clock.t += dt
    est.cb_altitude(m)


# ── 1. filter + ground reference + basic conversion ─────────────────────────
clock = Clock()
est, ekf = make_est(clock, gain=0.7, tau_s=1.0, ground_effect_alt=1.0, blend_s=2.0)
rng = np.random.default_rng(1)
H0 = 130.0
for _ in range(100):                       # 10 s on the ground, noise 0.085 RMS
    feed(est, H0 + rng.normal(0, 0.085), clock)
ekf["z"] = -6.0                            # EKF says -6 m on the ground (2026-09-13 style)
st = est.window_stats(5.0)
check("ground window filtered std small", st["std"] < 0.05, f"std={st['std']:.3f} spread={st['spread']:.3f} raw_std={st['raw_std']:.3f}")
est.set_ground(st["mean"], -6.0)
alt, mode = est.altitude()
check("on ground: alt ~0, EKF-LOCK", abs(alt) < 0.1 and mode == "EKF-LOCK", f"alt={alt:.3f} mode={mode}")
zsp = est.z_setpoint(2.0)
check("z_setpoint on ground = z_ekf + gain*2", abs(zsp - (-6.0 + 0.7 * 2.0)) < 0.1, f"zsp={zsp:.3f}")

# ── 2. climb: EKF-LOCK below 1 m, then BARO with a smooth switch ───────────
alts = []
modes = []
true_alt = 0.0
for i in range(400):                       # 40 s climb at 0.1 m/s, both sensors agree
    true_alt = min(3.0, i * 0.01)
    ekf["z"] = -6.0 + true_alt + 0.3 * math.sin(i * 0.01)   # slow EKF wander 0.3 m
    feed(est, H0 + true_alt + rng.normal(0, 0.085), clock)
    a, m = est.altitude()
    alts.append(a); modes.append(m)
alts = np.array(alts)
idx_baro = modes.index("BARO") if "BARO" in modes else None
check("switch to BARO once above ground-effect zone (~1.2 m est.)", idx_baro is not None and 80 < idx_baro < 200, f"first BARO at sample {idx_baro}")
steps = np.abs(np.diff(alts))
check("no step > 0.15 m at the mode switch (blend)", steps.max() < 0.15, f"max step {steps.max():.3f}")
check("in BARO the estimate ignores the EKF wander", abs(alts[-1] - 3.0) < 0.12, f"alt end={alts[-1]:.3f} (true 3.0, ekf wander 0.3)")

# ── 3. GPS jump in EKF z while in BARO: setpoint follows, altitude does not ──
z_before = est.z_setpoint(3.0)
ekf["z"] += 1.5                           # +1.5 m jump in the EKF
a_after, m_after = est.altitude()
z_after = est.z_setpoint(3.0)
check("EKF jump does not change the altitude estimate", abs(a_after - alts[-1]) < 0.02, f"{alts[-1]:.3f} -> {a_after:.3f}")
check("EKF jump shifts the setpoint by the same amount", abs((z_after - z_before) - 1.5) < 0.02, f"dz_sp={z_after - z_before:.3f}")

# ── 4. stale barometer -> EKF dead-reckoning from the fresh offset ──────────
clock.t += 1.5                            # no samples for 1.5 s
a_st, m_st = est.altitude()
check("stale -> STALE mode, continuous", m_st == "STALE" and abs(a_st - a_after) < 0.05, f"mode={m_st} alt={a_st:.3f}")
ekf["z"] += 0.4                           # real climb of 0.4 m seen by the EKF only
a_st2, _ = est.altitude()
check("stale: EKF motion is tracked", abs(a_st2 - (a_after + 0.4)) < 0.05, f"alt={a_st2:.3f}")
check("stale_for reports the gap", est.stale_for() > 1.4)
# samples return: STALE -> EKF-LOCK (dwell 1 s) -> BARO
for _ in range(3):
    feed(est, H0 + 3.4 + rng.normal(0, 0.085), clock)
a_back, m_back = est.altitude()
check("stream back -> EKF-LOCK first (dwell), continuous", m_back == "EKF-LOCK" and abs(a_back - a_st2) < 0.05, f"mode={m_back} alt={a_back:.3f}")
for _ in range(12):
    feed(est, H0 + 3.4 + rng.normal(0, 0.085), clock); est.altitude()
a_back, m_back = est.altitude()
check("stream back -> BARO after the dwell", m_back == "BARO", f"mode={m_back} alt={a_back:.3f}")

# ── 5. descent: re-lock below the ground-effect zone, no step ───────────────
prev = None
prev_mode = None
max_step = 0.0
switched_at = None
for i in range(400):                       # 40 s descent from 3.4 at 0.1 m/s
    true_alt = max(0.0, 3.4 - i * 0.01)
    ekf["z"] = -6.0 + true_alt + 0.5       # EKF offset changed by +0.5 during the flight
    ge = -0.3 * max(0.0, 1.0 - true_alt / 0.8)   # ground effect: baro reads LOW near the ground
    feed(est, H0 + true_alt + ge + rng.normal(0, 0.085), clock)
    a, m = est.altitude()
    if prev is not None:
        max_step = max(max_step, abs(a - prev))
    if m == "EKF-LOCK" and prev_mode == "BARO" and switched_at is None:
        switched_at = (i, a, true_alt)
    prev = a; prev_mode = m
check("re-lock to EKF below the zone", switched_at is not None and 0.8 < switched_at[2] < 1.0, f"switch at true alt {switched_at[2] if switched_at else None}")
check("descent: no step > 0.15 m", max_step < 0.15, f"max step {max_step:.3f}")
check("on the ground the estimate is ~0 despite ground effect", abs(prev) < 0.25, f"alt at touchdown {prev:.3f}")

# ── 6. glitch rejection ─────────────────────────────────────────────────────
est2, ekf2 = make_est(Clock(), tau_s=1.0)
c2 = Clock(); import takeoff_land_barometer.baro_altitude as ba; ba.time.time = c2
for _ in range(50):
    feed(est2, 100.0, c2)
feed(est2, 105.0, c2)                      # single 5 m outlier
check("single outlier rejected", abs(est2._h_filt - 100.0) < 0.01, f"filt={est2._h_filt:.3f} glitches={est2._n_glitch}")
for _ in range(5):
    feed(est2, 105.0, c2)                  # persistent step -> accepted
check("persistent step accepted after 3 samples", est2._h_filt > 100.5, f"filt={est2._h_filt:.3f}")

# ── 7. BaroZPublisher rewrites only position-z messages after the reference ──
class FakeNode:
    pass
class FakePub:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)
c3 = Clock(); ba.time.time = c3
holder = {"z": 5.0}
est3 = BaroAltitudeEstimator(None, lambda: holder["z"], gain=1.0, tau_s=0.5)
node = FakeNode(); node._baro = est3; node._ground_z = 5.0
pub = BaroZPublisher(FakePub(), node)
m = PositionTarget(); m.type_mask = PositionTarget.IGNORE_VX; m.position.z = 7.0
pub.publish(m)
check("before the reference: passthrough", pub._real.msgs[-1].position.z == 7.0)
for _ in range(30):
    feed(est3, 50.0, c3)
est3.set_ground(50.0, 5.0)
m = PositionTarget(); m.type_mask = PositionTarget.IGNORE_VX; m.position.z = 7.0   # nominal = ground_z + 2
pub.publish(m)
check("after the reference: z rewritten to z_ekf + min(gain*(2 - alt), max_cmd_offset 1.5)", abs(pub._real.msgs[-1].position.z - (5.0 + 1.5)) < 0.05, f"sent {pub._real.msgs[-1].position.z:.3f}")
holder["z"] = 5.0 + 2.0 + 0.8          # EKF says we are 0.8 m above target, baro says exactly at target
for _ in range(30):
    feed(est3, 52.0, c3)
# force BARO mode: climb the EKF-LOCK estimate above the zone, let the blend finish
est3.altitude()
for _ in range(40):
    feed(est3, 52.0, c3); est3.altitude()
m = PositionTarget(); m.type_mask = PositionTarget.IGNORE_VX; m.position.z = 7.0
pub.publish(m)
sent = pub._real.msgs[-1].position.z
alt, mode = est3.altitude()
check("BARO mode: EKF 0.8 m high but baro on target -> setpoint = z_ekf (hold)", mode == "BARO" and abs(sent - holder["z"]) < 0.1, f"mode={mode} alt={alt:.2f} sent={sent:.3f} z_ekf={holder['z']:.3f}")
m = PositionTarget(); m.type_mask = PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX; m.position.z = 7.0
pub.publish(m)
check("IGNORE_PZ messages untouched", pub._real.msgs[-1].position.z == 7.0)
check("rewrite counter (2 rewritten, 1 passthrough, 1 IGNORE_PZ)", pub.n_rewritten == 2, f"n={pub.n_rewritten}")

# ── 8. closed-loop simulation with recorded-like noise: drift + jump ────────
def simulate(drift_rate=0.0, jump_t=None, jump_m=0.0, gain=0.7, tau=1.0, dur=60.0, target=2.0, seed=3):
    """Plant: PX4 position loop P=1/s on the EKF z, vz limited to +-0.5 m/s,
    0.3 s velocity lag. EKF z = true z + gps bias. Baro = true + N(0,0.085)
    @10 Hz + ground effect. Returns (true altitude trace, max |err| after 15 s)."""
    ck = Clock(); ba.time.time = ck
    r = np.random.default_rng(seed)
    z_true, vz = 0.0, 0.0
    bias = -6.0
    hold = {"z": bias}
    e = BaroAltitudeEstimator(None, lambda: hold["z"], gain=gain, tau_s=tau, ground_effect_alt=1.0)
    for _ in range(100):
        feed(e, H0 + r.normal(0, 0.085), ck)
    e.set_ground(e.window_stats(5.0)["mean"], bias)
    dt = 0.02
    t = 0.0
    trace = []
    next_baro = 0.0
    while t < dur:
        if jump_t is not None and abs(t - jump_t) < dt / 2:
            bias += jump_m
        bias += drift_rate * dt
        hold["z"] = z_true + bias
        if t >= next_baro:
            ge = -0.3 * max(0.0, 1.0 - z_true / 0.8)
            m = Altitude(); m.monotonic = H0 + z_true + ge + r.normal(0, 0.085)
            e.cb_altitude(m)
            next_baro += 0.1
        zsp = e.z_setpoint(target)
        vz_cmd = max(-0.5, min(0.5, 1.0 * (zsp - hold["z"])))
        vz += (vz_cmd - vz) * dt / 0.3
        z_true = max(0.0, z_true + vz * dt)
        ck.t += dt
        t += dt
        trace.append((t, z_true, hold["z"], e.last_alt))
    tr = np.array(trace)
    settled = tr[tr[:, 0] > 15.0]
    return tr, float(np.abs(settled[:, 1] - target).max()), float(np.abs(settled[:, 1] - target).std())

tr, err, sd = simulate()
check("sim: nominal hover error < 0.15 m (noise 0.085 m RMS)", err < 0.15, f"max|err|={err:.3f} std={sd:.3f}")
tr, err, sd = simulate(drift_rate=-0.025)   # 1.5 m of GPS drift over 60 s (15:05 flight: 0.9 m/40 s)
check("sim: EKF drift -1.5 m/60 s -> physical altitude error < 0.15 m", err < 0.15, f"max|err|={err:.3f}; EKF z end={tr[-1,2]:.2f} (bias moved {tr[-1,2]-tr[-1,1]+6:.2f} m)")
tr, err, sd = simulate(jump_t=30.0, jump_m=+1.5)
check("sim: +1.5 m EKF jump at hover -> physical excursion < 0.2 m", err < 0.2, f"max|err|={err:.3f}")
tr, err, sd = simulate(jump_t=30.0, jump_m=-1.5)
check("sim: -1.5 m EKF jump at hover -> physical excursion < 0.2 m", err < 0.2, f"max|err|={err:.3f}")
tr, err, sd = simulate(gain=1.0, tau=1.0)
check("sim: gain 1.0 still stable (error < 0.2 m)", err < 0.2, f"max|err|={err:.3f} std={sd:.3f}")
tr, err, sd = simulate(gain=0.7, tau=1.5)
check("sim: tau 1.5 s still fine (error < 0.2 m)", err < 0.2, f"max|err|={err:.3f} std={sd:.3f}")
tr, err, sd = simulate(gain=0.7, tau=3.0)
check("sim: tau 3.0 s degrades (documented: keep baro_tau_s <= 1.5)", err > 0.2, f"max|err|={err:.3f} std={sd:.3f}")
tr, err, sd = simulate(gain=0.7, tau=1.0, seed=7)
check("sim: another noise seed, error < 0.15 m", err < 0.15, f"max|err|={err:.3f}")
t_reach = tr[np.argmax(tr[:, 1] > 1.85), 0]
check("sim: reaches 1.85 m within 12 s of the climb", t_reach < 12.0, f"t={t_reach:.1f}s")

n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} passed")
sys.exit(1 if n_fail else 0)
