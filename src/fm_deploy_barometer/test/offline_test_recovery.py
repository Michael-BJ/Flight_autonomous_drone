#!/usr/bin/env python3
"""Offline tests for ReturnHomeRecoveryMixin (stub host node, kinematic drone,
fake ESDF) and an instantiation test of FMInferenceBaroNode with the .onnx
model (no MAVROS, no camera, no arming).

    /usr/bin/python3 test_recovery.py
"""
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.expanduser("~/drone_ws/src/takeoff_land_barometer"))
sys.path.insert(0, os.path.expanduser("~/drone_ws/src/fm_deploy_barometer"))

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from mavros_msgs.msg import PositionTarget              # noqa: E402
from fm_deploy_barometer.recovery import ReturnHomeRecoveryMixin  # noqa: E402

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append((name, bool(cond), detail))
    print(("PASS " if cond else "FAIL ") + name + ("  " + detail if detail else ""))


class DS:
    def __init__(self):
        self.global_pos = np.zeros(3)
        self.global_vel = np.zeros(3)
        self.yaw = 0.0


class FakeESDF:
    def __init__(self):
        self.blocked = []      # list of (xy, radius): distance 0.1 inside
        self.ready = True
    def is_ready(self):
        return self.ready
    def get_edt_dis(self, p):
        p = np.asarray(p, dtype=float)
        for c, r in self.blocked:
            if np.linalg.norm(p - c) < r:
                return 0.1
        return 3.0


class FakeTraj:
    def __init__(self): self.n_inv = 0
    def invalidate(self): self.n_inv += 1


class RealStub(Node):
    """Stands in for FMInferenceRealNode BELOW the mixin: the methods the
    mixin wraps with super()."""
    STATE_IDLE = "IDLE"; STATE_TAKEOFF = "TAKEOFF"; STATE_FLYING = "FLYING"
    STATE_LANDING = "LANDING"; STATE_DONE = "DONE"

    def _land_and_finish(self):
        self.super_land_called += 1
        self._mission_state = self.STATE_LANDING

    def _publish_cmd(self):
        self.base_publish_called += 1


class Host(ReturnHomeRecoveryMixin, RealStub):
    """Minimal stand-in for FMInferenceRealNode: the attributes recovery.py
    touches, a kinematic drone that follows the published setpoint, and
    recording hooks."""

    def __init__(self):
        super().__init__("host_test")
        self.base_publish_called = 0
        self._mission_state = self.STATE_FLYING
        self._home_locked = True
        self._home_xy = np.array([0.0, 0.0])
        self._drone_state = DS()
        self._armed = True
        self._abort_reason = None
        self._global_target = np.array([6.0, 0.0])
        self._reached_target = False
        self._esdf = FakeESDF()
        self._traj = FakeTraj(); self._traj_lock = threading.Lock()
        self._max_home_d = 12.0; self._min_batt_v = 0.0; self._batt_v = 23.0
        self._min_batt_p = 0.0; self._batt_pct = 80.0
        self._dry_run = False; self._rc_override = False; self._link_lost = False
        self._kill_latched = False; self._disarm_abort = False; self._stream_on = True
        self._cruise_z = -4.0; self._hold_z = -6.0
        self.msgs = []
        self._pub_sp = self
        self.banners = []; self.notes = []
        self.super_land_called = 0
        self.hard_stops = []
        self.follow = True          # kinematic drone follows the setpoint
        self.follow_gain = 1.0
        self._rth_init("[RTH]")

    # publisher stand-in
    def publish(self, msg):
        self.msgs.append(msg)
    def _aborted(self):
        return self._rc_override or self._link_lost or self._kill_latched or self._disarm_abort
    def _announce_phase(self, name, detail="", warn=False):
        self.banners.append((name, detail))
    def _phase_note(self, text, warn=False):
        self.notes.append(text)
    def _hard_stop(self, reason, try_auto_land=False):
        self.hard_stops.append(reason)

    # simple physics: step toward the last setpoint at <= 0.6 m/s
    def step_physics(self, dt):
        if not self.follow or not self.msgs:
            return
        m = self.msgs[-1]
        tgt = np.array([m.position.x, m.position.y])
        p = self._drone_state.global_pos[:2]
        d = tgt - p
        n = np.linalg.norm(d)
        v = min(0.6, self.follow_gain * n)
        if n > 1e-6:
            p += d / n * v * dt
        self._drone_state.global_pos[:2] = p
        self._drone_state.yaw = float(m.yaw)


def drive(host, seconds, dt=0.05):
    """Spin the executor-free host: call _publish_cmd (what the cmd timer
    would do) and physics at 20 Hz while the RTH loop runs in its thread."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        host._publish_cmd()
        host.step_physics(dt)
        time.sleep(dt)


def lay_trail(host, pts):
    """Simulate FLYING along pts (the 5 Hz recorder is called directly)."""
    for p in pts:
        host._drone_state.global_pos[:2] = np.array(p, dtype=float)
        host._rth_record()


rclpy.init()

# ── R1. eligibility classification ──────────────────────────────────────────
h = Host()
lay_trail(h, [(0, 0), (1, 0), (2, 0), (3, 0), (3, 1), (3, 2)])
h._drone_state.global_pos[:2] = np.array([3.0, 2.0])
cases = [
    ("drone off-map / outside arena at (3.0,2.0) for 10s — unrecoverable without escape", True),
    ("STUCK at (3.0,2.0) — no progress toward the goal for 60s (still 3.6 m away)", True),
    (None, True),                                                  # mission timeout (goal not reached)
    ("GEOFENCE: 12.5 m from home (limit 12.0 m)", False),
    ("ALTITUDE deviated 0.60 m from cruise (limit 0.50 m)", False),
    ("BATTERY 20.1 V < 21.0 V", False),
    ("Unexpected DISARM while flying", False),
    ("BAROMETER STREAM LOST for 5.2s (limit 5s)", False),
    ("something new", False),
]
for reason, expect in cases:
    h._abort_reason = reason
    ok, why = h._rth_eligible()
    check(f"eligible({reason!r}) == {expect}", ok == expect, why)
h._abort_reason = None; h._reached_target = True
check("goal reached -> no return", h._rth_eligible()[0] is False, h._rth_eligible()[1])
h._reached_target = False
h._drone_state.global_pos[:2] = np.array([0.5, 0.0])
check("closer than rth_min_dist -> no return", h._rth_eligible()[0] is False, h._rth_eligible()[1])
h._drone_state.global_pos[:2] = np.array([3.0, 2.0]); h._armed = False
check("not armed -> no return", h._rth_eligible()[0] is False)
h._armed = True; h._kill_latched = True
check("kill latched -> no return", h._rth_eligible()[0] is False)
h._kill_latched = False
h.destroy_node()

# ── R2. return along an L-shaped trail, kinematic drone -> arrives home ─────
h = Host()
lay_trail(h, [(0, 0), (0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0),
              (3, 0.5), (3, 1), (3, 1.5), (3, 2)])
h._drone_state.global_pos[:2] = np.array([3.0, 2.0])
h._drone_state.yaw = math.pi / 2
h._abort_reason = "STUCK at (3.0,2.0) — no progress toward the goal for 60s"
res = {}
th = threading.Thread(target=lambda: res.update(r=h._land_and_finish()), daemon=True)
t0 = time.time(); th.start()
while th.is_alive() and time.time() - t0 < 90:
    drive(h, 0.05)
dur = time.time() - t0
p = h._drone_state.global_pos[:2]
check("R2: returned home", h._rth_result == "home" and np.linalg.norm(p) < 0.6, f"result={h._rth_result} pos=({p[0]:.2f},{p[1]:.2f}) in {dur:.1f}s")
check("R2: inherited landing called once after the return", h.super_land_called == 1)
check("R2: RETURN banner then 'landing at home'", any(b[0] == "RETURN" for b in h.banners) and any("landing at home" in n for n in h.notes), f"{h.banners[:1]} {h.notes[-1:]}")
check("R2: trajectory invalidated", h._traj.n_inv >= 1)
zs = {round(m.position.z, 2) for m in h.msgs}
check("R2: RETURN setpoints carry the nominal cruise z", zs == {-4.0}, f"{zs}")
check("R2: state back to LANDING", h._mission_state == "LANDING")
check("R2: outside RETURN the inherited _publish_cmd is used", h.base_publish_called > 0)
# path length ~5 m at 0.3 m/s -> >= 15 s, plus yaw turn and hover 2 s
check("R2: took a plausible time (>= 15 s, < 60 s)", 15.0 <= dur < 60.0, f"{dur:.1f}s")
yaws = [m.yaw for m in h.msgs]
check("R2: yaw slewed (no 180 deg jump in one tick)", max(abs((b - a + math.pi) % (2 * math.pi) - math.pi) for a, b in zip(yaws[:-1], yaws[1:])) < math.radians(15), "")
h.destroy_node()

# ── R3. blocked path -> holds, then lands in place after rth_block_s ────────
h = Host()
lay_trail(h, [(0, 0), (0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0)])
h._drone_state.global_pos[:2] = np.array([3.0, 0.0])
h._esdf.blocked = [(np.array([1.5, 0.0]), 0.4)]      # a new obstacle on the way back
h._abort_reason = "drone off-map / outside arena at (3.0,0.0) for 10s — unrecoverable without escape"
h.set_parameters([rclpy.parameter.Parameter("rth_block_s", value=3.0)]); h._rth_block_s = 3.0
th = threading.Thread(target=h._land_and_finish, daemon=True); t0 = time.time(); th.start()
while th.is_alive() and time.time() - t0 < 60:
    drive(h, 0.05)
p = h._drone_state.global_pos[:2]
check("R3: stopped by the obstacle, landed in place", h._rth_result and "blocked" in h._rth_result and 1.7 < p[0] < 3.1, f"result={h._rth_result} pos=({p[0]:.2f},{p[1]:.2f})")
check("R3: inherited landing still called", h.super_land_called == 1)
h.destroy_node()

# ── R4. drone does not follow (wind / GPS) -> tracking-error stop ───────────
h = Host()
lay_trail(h, [(0, 0), (0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0)])
h._drone_state.global_pos[:2] = np.array([3.0, 0.0])
h.follow = False
h._abort_reason = "STUCK at (3.0,0.0) — no progress"
h._rth_lead_max = 5.0     # let the carrot run away so the tracking error builds
th = threading.Thread(target=h._land_and_finish, daemon=True); t0 = time.time(); th.start()
while th.is_alive() and time.time() - t0 < 60:
    drive(h, 0.05)
check("R4: tracking-error stop", h._rth_result and "behind the carrot" in h._rth_result, f"result={h._rth_result} in {time.time()-t0:.1f}s")
h.destroy_node()

# ── R5. kill switch during the return -> hard stop, no landing sequence ─────
h = Host()
lay_trail(h, [(0, 0), (0.5, 0), (1, 0), (1.5, 0), (2, 0), (2.5, 0), (3, 0)])
h._drone_state.global_pos[:2] = np.array([3.0, 0.0])
h._abort_reason = "STUCK at (3.0,0.0) — no progress"
th = threading.Thread(target=h._land_and_finish, daemon=True); t0 = time.time(); th.start()
drive(h, 3.0)
n_before = len(h.msgs)
h._kill_latched = True
drive(h, 1.0)
th.join(5.0)
check("R5: kill -> _hard_stop, inherited landing NOT called", h.hard_stops and h.super_land_called == 0, f"{h.hard_stops}")
check("R5: no setpoint published after the kill", len(h.msgs) == n_before, f"{len(h.msgs)} vs {n_before}")
h.destroy_node()

# ── R6. rth_enabled false -> straight to the inherited landing ──────────────
rclpy.shutdown()
rclpy.init(args=["--ros-args", "-p", "rth_enabled:=false"])
h = Host()
lay_trail(h, [(0, 0), (1, 0), (2, 0), (3, 0)])
h._drone_state.global_pos[:2] = np.array([3.0, 0.0])
h._abort_reason = "STUCK at (3.0,0.0)"
h._land_and_finish()
check("R6: rth_enabled:=false -> land in place immediately", h.super_land_called == 1 and not h._rth_ran and not h.banners)
h.destroy_node()
rclpy.shutdown()

# ── R7. FMInferenceBaroNode instantiation (onnx, dry_run, no MAVROS) ────────
onnx = os.path.expanduser("~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx")
rclpy.init(args=["--ros-args", "-p", f"model_path:={onnx}", "-p", "dry_run:=true",
                 "-p", "max_alt_error:=0.5", "-p", "target_alt:=2.0", "-p", "color_output:=false"])
try:
    from fm_deploy_barometer.fm_inference_baro_node import FMInferenceBaroNode
    from takeoff_land_barometer.baro_altitude import BaroZPublisher
    n = FMInferenceBaroNode()
    check("R7: node constructed", True)
    check("R7: publisher wrapped", isinstance(n._pub_sp, BaroZPublisher))
    check("R7: inherited EKF-z altitude test disabled, baro limit kept", n._max_alt_err == 0.0 and n._baro_max_alt_err == 0.5)
    check("R7: rth params + trail", n._rth_enabled and n._trail == [] and n._rth_speed == 0.3)
    n._watchdog(); n._print_status(); n._rth_record()
    check("R7: watchdog / status / recorder run before any data", True)
    ok, why = n._rth_eligible()
    check("R7: not eligible before flight", ok is False, why)
    # publish path: a nominal setpoint before the reference passes through
    n._hold_z = n._ground_z = 0.0
    n._publish_cmd()
    check("R7: _publish_cmd runs (dry_run gate)", True)
    n.destroy_node()
except Exception as e:
    check("R7: node constructed", False, repr(e))
finally:
    if rclpy.ok():
        rclpy.shutdown()

n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
print(f"\n{len(RESULTS) - n_fail}/{len(RESULTS)} passed")
sys.exit(1 if n_fail else 0)
