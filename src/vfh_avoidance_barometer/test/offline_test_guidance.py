#!/usr/bin/env python3
"""Tick-level tests of VFHFlightBaroNode._fly_vfh and the perception node's
ground filter. No MAVROS, nothing spins, nothing arms: the node's state
(_pos, _yaw, _armed, the VFH message) is set directly and _fly_vfh runs in
a thread for a moment while we watch the setpoint it writes.

Run under the ROS env with /usr/bin/python3 (the packages are imported from
src, the parents from install or src)."""
import math
import os
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
for pkg in ("forward_move", "takeoff_land", "takeoff_land_barometer"):
    sys.path.insert(0, os.path.expanduser(f"~/drone_ws/src/{pkg}"))

import rclpy                                                        # noqa: E402
from vfh_avoidance_barometer.vfh_flight_baro_node import VFHFlightBaroNode, _wrap   # noqa: E402
from vfh_avoidance_barometer.vfh_avoidance_node import VFHAvoidanceNode             # noqa: E402
from offline_test_vfh_core import render_scene, FX, FY, H, W                         # noqa: E402

FAILS = 0
N = 0


def check(name, cond, info=""):
    global FAILS, N
    N += 1
    print(f"  [{'OK ' if cond else 'FAIL'}] {name} {info}")
    if not cond:
        FAILS += 1


class Harness:
    """Runs _fly_vfh in a thread against injected state."""

    def __init__(self, node, yaw0=0.0, goal_dist=3.0):
        self.node = node
        n = node
        n._goal_dist = goal_dist; n._fwd_dist = goal_dist
        n._launch_x, n._launch_y = 0.0, 0.0
        n._home_x, n._home_y, n._home_yaw = 0.0, 0.0, yaw0
        n._ground_z = 0.0
        n._pos = np.array([0.0, 0.0, 2.0]); n._vel = np.zeros(3); n._yaw = 0.0
        n._armed = True; n._connected = True
        n._kill_latched = n._disarm_abort = n._rc_override = n._link_lost = False
        n._leg_result = ""
        n._stream_on = True
        n._max_pos_error = 2.0
        with n._vfh_lock:
            n._vfh_msg = None; n._vfh_t = None
        self.targets = []
        n._pub_target.publish = lambda m: self.targets.append(float(m.data))
        self.aborts = []
        n._sanity_abort = lambda reason: (self.aborts.append(reason),
                                          setattr(n, "_stream_on", False))
        self.result = None
        self.th = None
        self._feed = None
        self._stop = False

    def feed(self, msg):
        """Keep this VFH message fresh (re-stamped every 50 ms) until changed."""
        self._feed = msg

    def start(self, z=2.0):
        def run():
            self.result = self.node._fly_vfh(z)
        self.th = threading.Thread(target=run, daemon=True)
        self.th.start()
        def feeder():
            while not self._stop and self.th.is_alive():
                if self._feed is not None:
                    with self.node._vfh_lock:
                        self.node._vfh_msg = dict(self._feed)
                        self.node._vfh_t = time.monotonic()
                time.sleep(0.05)
        threading.Thread(target=feeder, daemon=True).start()

    def stop(self):
        self._stop = True
        self.node._kill_latched = True       # makes the loop return False
        self.th.join(timeout=3.0)
        self.node._kill_latched = False

    def sp(self):
        n = self.node
        with n._sp_lock:
            return n._sp_x, n._sp_y, n._sp_z, n._sp_yaw


def new_node(**over):
    params = dict(baro_stale_abort_s=0.0, max_alt_error=100.0, max_pos_error=2.0,
                  max_vz=100.0, cmd_hz=50, color_output=False, status_period_s=100.0,
                  verify_rc_override_param=False, require_rc_offboard=False)
    params.update(over)
    argv = ["--ros-args"]
    for k, v in params.items():
        argv += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
    rclpy.init(args=argv)
    return VFHFlightBaroNode()


def main():
    print("== _wrap ==")
    check("wrap 190 deg -> -170", abs(_wrap(math.radians(190)) - math.radians(-170)) < 1e-9)
    check("wrap -190 deg -> +170", abs(_wrap(math.radians(-190)) - math.radians(170)) < 1e-9)

    print("== steering / target signs (yaw 0, goal 3 m ahead) ==")
    node = new_node()
    MOVE = {"state": "move", "steer_deg": 0.0, "front_min_m": None, "avoid_toward": "none"}

    h = Harness(node); h.feed(dict(MOVE, steer_deg=20.0)); h.start(); time.sleep(1.2)
    x, y, z, yaw = h.sp(); h.stop()
    check("steer +20 (right) -> yaw setpoint NEGATIVE (CW)", -0.40 < yaw < -0.30,
          f"yaw_sp={math.degrees(yaw):.1f} deg")
    check("   and the setpoint advanced along the heading", x > 0.15 and y < 0.0,
          f"sp=({x:.2f},{y:.2f})")
    check("   z setpoint = cruise z", abs(z - 2.0) < 1e-9)
    check("   target angle sent ~0 (goal straight ahead)", h.targets and abs(_wrap(math.radians(h.targets[-1]))) < 0.05,
          f"{h.targets[-1] if h.targets else None}")
    check("   result None (still flying when stopped)", h.result is False)

    h = Harness(node); h.feed(dict(MOVE, steer_deg=-20.0)); h.start(); time.sleep(1.2)
    x, y, z, yaw = h.sp(); h.stop()
    check("steer -20 (left) -> yaw setpoint POSITIVE (CCW)", 0.30 < yaw < 0.40, f"{math.degrees(yaw):.1f}")
    check("   setpoint moved left of the line", y > 0.0 and x > 0.15, f"sp=({x:.2f},{y:.2f})")

    h = Harness(node); h.feed(dict(MOVE, steer_deg=40.0)); h.start(); time.sleep(0.5)
    x, y, z, yaw = h.sp()
    check("yaw slew limited to 30 deg/s", -0.30 < yaw < -0.17, f"{math.degrees(yaw):.1f} deg after 0.5 s")
    time.sleep(0.6); x1, _, _, yaw1 = h.sp(); time.sleep(0.5); x2, _, _, yaw2 = h.sp(); h.stop()
    check("   XY frozen once the heading error > 25 deg (yaw keeps turning)",
          abs(x2 - x1) < 1e-6 and 0.0 < x1 < 0.35 and yaw2 < yaw1 + 1e-9 and yaw2 < -0.6,
          f"x 1.1s={x1:.2f} 1.6s={x2:.2f} yaw {math.degrees(yaw1):.0f}->{math.degrees(yaw2):.0f}")

    h = Harness(node); h.feed({"state": "stop", "steer_deg": 0.0, "front_min_m": 1.5,
                               "avoid_toward": "left"}); h.start(); time.sleep(1.0)
    x, y, z, yaw = h.sp(); h.stop()
    check("state stop -> XY frozen", abs(x) < 1e-6 and abs(y) < 1e-6, f"sp=({x:.2f},{y:.2f})")
    check("   yaw searches toward avoid_toward=left (+) at half rate", 0.15 < yaw < 0.35,
          f"{math.degrees(yaw):.1f} deg after 1 s")

    h = Harness(node); h.feed(dict(MOVE, steer_deg=10.0, front_min_m=0.8)); h.start(); time.sleep(1.0)
    x, y, z, yaw = h.sp(); h.stop()
    check("front_min 0.8 < stop_dist 1.2 -> hard brake, XY frozen", abs(x) < 1e-6, f"sp x={x:.2f}")
    check("   but the yaw still turns away", yaw < -0.15, f"{math.degrees(yaw):.1f}")

    h = Harness(node); h.start(); time.sleep(1.0)            # never any VFH message
    x, y, z, yaw = h.sp()
    check("no VFH message -> XY frozen, yaw unchanged", abs(x) < 1e-6 and abs(yaw) < 1e-6)
    node._vfh_abort_s = 1.5
    time.sleep(1.5); h.th.join(timeout=2.0)
    check("   no VFH for > vfh_stale_abort_s -> leg ends True (land here)",
          h.result is True and "perception lost" in node._leg_result, node._leg_result)
    node._vfh_abort_s = 5.0

    h = Harness(node); h.feed(MOVE); node._pos = np.array([2.7, 0.1, 2.0]); node._max_pos_error = 10.0
    h.start(); h.th.join(timeout=2.0)
    check("drone within goal_tol -> True, 'goal reached'", h.result is True and "goal reached" in node._leg_result,
          node._leg_result)

    h = Harness(node); h.feed(MOVE); node._pos = np.array([5.5, 0.0, 2.0]); node._max_pos_error = 10.0
    h.start(); h.th.join(timeout=2.0)
    check("beyond goal_dist + geofence_margin -> sanity abort, False",
          h.result is False and h.aborts and "GEOFENCE" in h.aborts[0], str(h.aborts))

    h = Harness(node); h.feed(MOVE); node._pos = np.array([2.5, 0.0, 2.0]); h.start(); h.th.join(timeout=2.0)
    check("tracking error > max_pos_error -> inherited sanity abort, False",
          h.result is False and h.aborts and "drift" in h.aborts[0], str(h.aborts))

    h = Harness(node); h.feed(MOVE); node._kill_latched = True; h.start(); h.th.join(timeout=2.0)
    check("kill latched -> False immediately", h.result is False)
    node._kill_latched = False

    h = Harness(node); h.feed(MOVE); node._armed = False; h.start(); h.th.join(timeout=2.0)
    check("disarmed -> False", h.result is False)

    h = Harness(node); h.feed(MOVE); node._mission_to = 0.5; h.start(); h.th.join(timeout=3.0)
    check("mission timeout -> True, land here", h.result is True and "timeout" in node._leg_result, node._leg_result)
    node._mission_to = 120.0

    print("== goal bearing -> VFH target angle ==")
    h = Harness(node, yaw0=math.radians(40.0)); h.feed(MOVE); h.start(); time.sleep(0.4)
    tgt = h.targets[-1] if h.targets else None; h.stop()
    check("goal 40 deg LEFT of the nose -> target 320 (= -40, VFH left)", tgt is not None and abs(tgt - 320.0) < 1.0, str(tgt))
    h = Harness(node, yaw0=math.radians(-40.0)); h.feed(MOVE); h.start(); time.sleep(0.4)
    tgt = h.targets[-1] if h.targets else None; h.stop()
    check("goal 40 deg RIGHT of the nose -> target 40", tgt is not None and abs(tgt - 40.0) < 1.0, str(tgt))
    h = Harness(node, yaw0=math.radians(40.0)); h.feed(MOVE); h.start(); time.sleep(0.6)
    x, y, z, yaw = h.sp(); h.stop()
    check("goal 40 deg left but VFH steer 0: the drone follows VFH (straight along the nose)",
          x > 0.1 and abs(y) < 1e-6 and abs(yaw) < 1e-9,
          f"sp=({x:.2f},{y:.2f}) yaw_sp={math.degrees(yaw):.1f} (a real VFH returns steer -40 for target 320)")
    h = Harness(node, yaw0=math.radians(40.0)); h.feed(dict(MOVE, steer_deg=-40.0)); h.start(); time.sleep(2.0)
    x, y, z, yaw = h.sp(); h.stop()
    check("   with steer -40 the yaw turns left (+40) and the setpoint moves toward the goal",
          abs(yaw - math.radians(40.0)) < 0.02 and y > 0.03 and x > 0.05,
          f"sp=({x:.2f},{y:.2f}) yaw_sp={math.degrees(yaw):.1f}")
    h = Harness(node); h.feed(MOVE); node._yaw = math.radians(-30.0); h.start(); time.sleep(0.3)
    x, y, z, yaw = h.sp(); h.stop()
    check("yaw setpoint starts at the REAL heading, not the launch heading",
          abs(yaw - math.radians(-30.0)) < 0.02, f"yaw_sp={math.degrees(yaw):.1f}")
    node._yaw = 0.0

    h = Harness(node, yaw0=math.radians(100.0)); h.feed(MOVE); h.start(); time.sleep(1.0)
    x, y, z, yaw = h.sp(); h.stop()
    check("goal 100 deg off the nose (> goal_fov_half 80): XY frozen, yaw turns toward it",
          abs(x) < 1e-6 and abs(y) < 1e-6 and 0.4 < yaw < 0.6, f"sp=({x:.2f},{y:.2f}) yaw_sp={math.degrees(yaw):.1f}")

    print("== parameter validation ==")
    node._goal_dist = 31.0
    check("goal_dist 31 rejected", node._forward_params_problem() is not None)
    node._goal_dist = 3.0; node._goal_tol = 3.5
    check("goal_tol >= goal_dist rejected", node._forward_params_problem() is not None)
    node._goal_tol = 0.5; node._max_lead = 5.0
    check("max_lead > max_pos_error rejected", node._forward_params_problem() is not None)
    node._max_lead = 0.6
    check("defaults accepted", node._forward_params_problem() is None, str(node._forward_params_problem()))
    node._fwd_speed = 2.0
    check("forward_speed 2.0 rejected by the parent check", node._forward_params_problem() is not None)
    node._fwd_speed = 0.3

    print("== AGL publication ==")
    got = []
    node._pub_agl.publish = lambda m: got.append(float(m.data))
    node._publish_agl()
    check("before the ground reference: agl 0.0", got == [0.0], str(got))

    node.destroy_node(); rclpy.shutdown()

    print("== perception node: barometric ground filter ==")
    rclpy.init(args=["--ros-args", "-p", "publish_visual:=false"])
    vn = VFHAvoidanceNode()
    from sensor_msgs.msg import CameraInfo
    ci = CameraInfo(); ci.width = W; ci.height = H
    ci.k = [FX, 0.0, W / 2.0, 0.0, FY, H / 2.0, 0.0, 0.0, 1.0]
    vn._info_callback(ci)
    check("camera_info -> hfov 91 from fx", abs(vn._vfh.hfov_deg - 91.0) < 0.1, f"{vn._vfh.hfov_deg:.1f}")
    d = render_scene([], ground=True, alt_m=2.0)
    st = vn._apply_ground_filter(d.copy())
    check("no AGL yet -> filter inactive ('no-agl')", st == "no-agl", st)
    from std_msgs.msg import Float32
    vn._agl_callback(Float32(data=2.0))
    d2 = d.copy(); st = vn._apply_ground_filter(d2)
    r = vn._vfh.run(d2, 0.0)
    check("AGL 2.0: open field with ground -> move (ground removed)", st.startswith("agl") and r["cmd"]["state"] == "move",
          f"{st} {r['cmd']}")
    d3 = render_scene([(2.5, 0.8, 0.3)], ground=True, alt_m=2.0); vn._apply_ground_filter(d3)
    r = vn._vfh.run(d3, 0.0)
    check("AGL 2.0: pole LEFT survives the filter -> steer > 0", r["steer_deg"] > 0, f"{r['steer_deg']} {r['cmd']}")
    d4 = render_scene([(2.5, y, 0.35) for y in np.arange(-3.0, 3.01, 0.3)], ground=True, alt_m=2.0)
    vn._apply_ground_filter(d4); r = vn._vfh.run(d4, 0.0)
    check("AGL 2.0: wall survives -> stop", r["cmd"]["state"] == "stop", str(r["cmd"]))
    # a low obstacle (0.5 m tall box) is kept only above agl - margin: with agl 2.0,
    # margin 0.5, anything more than 1.5 m below the camera is dropped -> a 0.5 m
    # box top is 1.5 m below -> dropped. That is the documented limit.
    vn._agl_callback(Float32(data=0.0))
    d5 = render_scene([(2.5, 0.8, 0.3)], ground=False, alt_m=0.0); vn._apply_ground_filter(d5)
    r = vn._vfh.run(d5, 0.0)
    check("AGL 0 (on the ground): pole still seen at the horizon rows", r["cmd"]["state"] != "move",
          str(r["cmd"]))
    vn.destroy_node(); rclpy.shutdown()

    print(f"\n{N - FAILS}/{N} passed")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
