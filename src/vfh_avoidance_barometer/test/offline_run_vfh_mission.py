#!/usr/bin/env python3
"""Run ONE closed-loop VFH mission offline: fake PX4 (takeoff_land_barometer's)
+ synthetic depth world + the REAL vfh_avoidance_node + the REAL
vfh_flight_baro_node, all in one process. No hardware, no MAVROS, nothing
arms. Prints a RESULT json line.

usage: offline_run_vfh_mission.py <scenario>
"""
import json
import math
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.expanduser("~/drone_ws/src/takeoff_land_barometer/test"))
for pkg in ("forward_move", "takeoff_land", "takeoff_land_barometer"):
    sys.path.insert(0, os.path.expanduser(f"~/drone_ws/src/{pkg}"))

import rclpy                                          # noqa: E402
from rclpy.executors import MultiThreadedExecutor      # noqa: E402
from offline_fake_px4 import FakePX4                   # noqa: E402
from offline_fake_world import DepthWorld              # noqa: E402

# the fake PX4 yaw is 0.3 rad at start (see offline_fake_px4.py); the goal
# lies along it. Cylinders are given in the LAUNCH frame (ahead, left) and
# rotated into ENU here.
YAW0 = 0.3


def ahead_left(a, l, r):
    c, s = math.cos(YAW0), math.sin(YAW0)
    return (a * c - l * s, a * s + l * c, r)


BASE = dict(target_alt=2.0, hover_time=4.0, max_pos_error=1.5, max_vz=1.0, max_alt_error=0.6,
            forward_speed=0.4, forward_hold_time=2.0, goal_tol=0.5, stop_dist=1.2)

SCENARIOS = {
    # open field, goal 4 m: straight there, DONE at the goal
    "open":        dict(goal_dist=4.0, cyl=[]),
    # a 0.3 m pole right on the line 2.5 m out: steer around it, reach the goal
    "pole_center": dict(goal_dist=5.0, cyl=[ahead_left(2.5, 0.0, 0.3)]),
    # pole 0.5 m LEFT of the line -> should pass on the right
    "pole_left":   dict(goal_dist=5.0, cyl=[ahead_left(2.5, 0.5, 0.3)]),
    # pole 0.5 m RIGHT of the line -> should pass on the left
    "pole_right":  dict(goal_dist=5.0, cyl=[ahead_left(2.5, -0.5, 0.3)]),
    # two poles forming a 1.6 m gap slightly left of the line
    "gate":        dict(goal_dist=6.0, cyl=[ahead_left(3.0, 1.1, 0.25), ahead_left(3.0, -0.9, 0.25)]),
    # a wide wall 3 m out: no gap -> BLOCKED -> land in place, never closer than stop_dist-ish
    "wall":        dict(goal_dist=6.0, blocked_timeout_s=8.0,
                       cyl=[ahead_left(3.0, l, 0.35) for l in [x * 0.5 for x in range(-12, 13)]]),
    # depth frames stop mid-leg -> perception lost -> land here
    "vfh_lost":    dict(goal_dist=6.0, stop_frames_after=40.0, vfh_stale_abort_s=3.0, cyl=[]),
    # kill switch during the leg (inherited handler)
    "kill":        dict(goal_dist=6.0, kill_at=44.0, cyl=[]),
}


def main():
    name = sys.argv[1]
    sc = SCENARIOS[name]
    params = dict(gps_stable_dur=2.0, gps_wait_timeout=30.0, connect_timeout=30.0,
                  status_period_s=5.0, color_output=False, cmd_hz=50,
                  publish_visual=False, log_period_s=2.0)
    params.update(BASE)
    params.update({k: v for k, v in sc.items() if k not in ("cyl", "stop_frames_after", "kill_at")})
    argv = ["--ros-args"]
    for k, v in params.items():
        argv += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    rclpy.init(args=argv)

    fake_sc = dict(ekf_drift_rate=-0.01, ekf_wander_amp=0.1)
    if "kill_at" in sc:
        fake_sc["kill_at"] = sc["kill_at"]
    fake = FakePX4(fake_sc)
    world = DepthWorld(fake, sc["cyl"], stop_after=sc.get("stop_frames_after"))

    from vfh_avoidance_barometer.vfh_avoidance_node import VFHAvoidanceNode
    from vfh_avoidance_barometer.vfh_flight_baro_node import VFHFlightBaroNode
    vfh = VFHAvoidanceNode()
    node = VFHFlightBaroNode()

    phases = []
    orig_announce = node._announce_phase
    def _ann(n, detail="", warn=False):
        phases.append((round(fake.T(), 1), n))
        return orig_announce(n, detail, warn)
    node._announce_phase = _ann

    ex = MultiThreadedExecutor(num_threads=6)
    for n in (fake, world, vfh, node):
        ex.add_node(n)
    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()
    t0 = time.time()
    track = []
    t_last = 0.0
    while rclpy.ok() and th.is_alive() and time.time() - t0 < 300.0:
        ex.spin_once(timeout_sec=0.05)
        if time.time() - t_last > 0.5:
            t_last = time.time()
            track.append((round(fake.T(), 1), round(fake.x, 2), round(fake.y, 2), round(fake.z_true, 2),
                          round(math.degrees(fake.yaw), 0), node._phase_name))
    t1 = time.time()
    while rclpy.ok() and time.time() - t1 < 0.5:
        ex.spin_once(timeout_sec=0.05)

    # launch-frame coordinates of the end point
    c, s = math.cos(YAW0), math.sin(YAW0)
    ahead = fake.x * c + fake.y * s
    left = -fake.x * s + fake.y * c
    goal_dist = params["goal_dist"]
    out = {
        "scenario": name,
        "phase": node._phase_name,
        "mission": node._mission,
        "leg_result": node._leg_result,
        "armed_end": fake.armed,
        "mode_end": fake.mode,
        "events": fake.log,
        "end_ahead_left": [round(ahead, 2), round(left, 2)],
        "dist_to_goal_end": round(math.hypot(ahead - goal_dist, left), 2),
        "min_clearance_m": None if world.min_clearance == float("inf") else round(world.min_clearance, 2),
        "min_clearance_at": world.min_clearance_at,
        "max_true_alt": round(fake.max_true_alt, 2),
        "z_true_end": round(fake.z_true, 2),
        "depth_frames": world.n_frames,
        "vfh_msgs": node._vfh_n,
        "kill_latched": node._kill_latched,
        "stream_on": node._stream_on,
        "phases": phases,
        "track": track[::4],
    }
    print("RESULT " + json.dumps(out))
    try:
        for n in (node, vfh, world, fake):
            n.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
