#!/usr/bin/env python3
"""Run ONE offline mission scenario: fake PX4 + a real flight node class in
the same process. Prints a JSON summary line at the end (prefix RESULT).

usage: run_mission.py <scenario-name>
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.expanduser("~/drone_ws/src/takeoff_land_barometer"))
sys.path.insert(0, os.path.expanduser("~/drone_ws/src/forward_move_barometer"))

import rclpy                                   # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from offline_fake_px4 import FakePX4                   # noqa: E402

SCENARIOS = {
    # takeoff_land_barometer, slow EKF drift + wander (old gate would refuse: spread 0.4 m/5 s? no —
    # wander 0.25 m amp / 30 s period gives ~0.1 m per 5 s; the drift is what matters in flight)
    "tl_nominal": dict(node="tl", ekf_drift_rate=-0.025, ekf_wander_amp=0.15, params=dict(
        target_alt=2.0, hover_time=10.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5)),
    # EKF jumps +1.5 m during hover and -1.5 m later
    "tl_jump": dict(node="tl", ekf_jumps=[(36.0, +1.5), (42.0, -2.5)], params=dict(
        target_alt=2.0, hover_time=20.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5)),
    # barometer stream dies during hover -> expect sanity abort AUTO.LAND
    "tl_baro_lost": dict(node="tl", baro_off_at=50.0, params=dict(
        target_alt=2.0, hover_time=20.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5)),
    # kill switch during takeoff (inherited path must still work)
    "tl_kill": dict(node="tl", kill_at=40.0, params=dict(
        target_alt=2.0, hover_time=10.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5)),
    # forward_move_barometer nominal with drift
    "fm_nominal": dict(node="fwd", ekf_drift_rate=+0.02, params=dict(
        target_alt=2.0, hover_time=5.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5,
        forward_distance=1.5, forward_speed=0.3, forward_hold_time=2.0)),
    # EKF z far from zero and jittery-but-slow: barometer package must accept, gate on baro
    "tl_bigoffset": dict(node="tl", ekf_bias0=-45.0, ekf_wander_amp=0.2, ekf_wander_period=40.0, params=dict(
        target_alt=1.5, hover_time=6.0, max_pos_error=1.0, max_vz=1.0, max_alt_error=0.5)),
}


def main():
    name = sys.argv[1]
    sc = SCENARIOS[name]
    params = dict(gps_stable_dur=2.0, gps_wait_timeout=30.0, connect_timeout=30.0,
                  status_period_s=5.0, color_output=False, cmd_hz=50)
    params.update(sc["params"])
    argv = ["--ros-args"]
    for k, v in params.items():
        argv += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    rclpy.init(args=argv)
    fake = FakePX4(sc)
    if sc["node"] == "tl":
        from takeoff_land_barometer.takeoff_land_baro_node import TakeoffLandBaroNode
        node = TakeoffLandBaroNode()
    else:
        from forward_move_barometer.forward_move_baro_node import ForwardMoveBaroNode
        node = ForwardMoveBaroNode()

    phases = []
    orig_announce = node._announce_phase
    def _ann(name, detail="", warn=False):
        phases.append((fake.T(), name))
        return orig_announce(name, detail, warn)
    node._announce_phase = _ann
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(fake); ex.add_node(node)
    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()
    t0 = time.time()
    while rclpy.ok() and th.is_alive() and time.time() - t0 < 240.0:
        ex.spin_once(timeout_sec=0.05)
    # drain a little
    t1 = time.time()
    while rclpy.ok() and time.time() - t1 < 0.5:
        ex.spin_once(timeout_sec=0.05)

    # summarise
    samples = fake.samples
    t_h = next((t for t, n in phases if n == "HOVER"), None)
    t_l = next((t for t, n in phases if n in ("LANDING", "ABORT", "FORWARD")), None)
    hover = [s for s in samples if t_h is not None and t_l is not None and t_h + 1.0 <= s[0] <= t_l]
    alt_target = params["target_alt"]
    hov_err = max((abs(s[3] - alt_target) for s in hover), default=float("nan"))
    ekf_span = (max(s[4] for s in hover) - min(s[4] for s in hover)) if hover else float("nan")
    out = {
        "scenario": name,
        "phase": node._phase_name,
        "mission": node._mission,
        "armed_end": fake.armed,
        "mode_end": fake.mode,
        "events": fake.log,
        "max_true_alt": round(fake.max_true_alt, 3),
        "hover_max_alt_err": round(hov_err, 3),
        "ekf_z_span_in_hover": round(ekf_span, 3),
        "z_true_end": round(fake.z_true, 3),
        "ground_z_ekf": round(node._ground_z, 3),
        "baro_ready": node._baro.ready,
        "mode_switches": node._baro.n_mode_switch,
        "rewritten": node._pub_sp.n_rewritten,
        "kill_latched": node._kill_latched,
        "stream_on": node._stream_on,
        "trail_end_xy": [round(fake.x, 2), round(fake.y, 2)],
        "phases": [(round(t, 1), n) for t, n in phases],
        "hover_window": [round(t_h, 1) if t_h else None, round(t_l, 1) if t_l else None],
        "hover_true_alt_min_max": [round(min(s[3] for s in hover), 3), round(max(s[3] for s in hover), 3)] if hover else None,
    }
    print("RESULT " + json.dumps(out))
    try:
        node.destroy_node(); fake.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
