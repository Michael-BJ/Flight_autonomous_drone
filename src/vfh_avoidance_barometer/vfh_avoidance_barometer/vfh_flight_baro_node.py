#!/usr/bin/env python3
"""
vfh_flight_baro_node.py
==================================================================
PURPOSE: TAKE OFF -> HOVER -> fly to a goal `goal_dist` m straight ahead
WHILE STEERING AROUND OBSTACLES with VFH (depth camera) -> HOLD -> LANDING
where the drone ends up. The ALTITUDE is measured with the BAROMETER
(x/y stay GPS/EKF), exactly like forward_move_barometer.

NOT A COPY:
    VFHFlightBaroNode = TakeoffLandBaroMixin + ForwardMoveNode.
    forward_move's run_sequence (PREFLIGHT -> READY -> ARMING -> TAKEOFF ->
    HOVER -> FORWARD -> HOLD -> LANDING -> DONE), every pre-flight gate,
    the ARM/OFFBOARD handshake, every abort path (kill switch, unexpected
    disarm, RC override, link loss, sanity -> AUTO.LAND) and the barometric
    altitude loop are the inherited code. This file replaces ONE method:

        _fly_forward(z)   the FORWARD leg  ->  VFH-guided leg (_fly_vfh)

    plus a pre-flight check that the VFH perception stream is alive
    (_wait_ekf_stable is wrapped for that) and two cosmetic overrides that
    rewrite forward_move's "straight ahead / NO OBSTACLE AVOIDANCE" banner
    texts. When forward_move_node.run_sequence() changes, this node follows
    automatically after a rebuild.

VFH-GUIDED LEG (see _fly_vfh):
    goal  = launch point + goal_dist * (cos yaw0, sin yaw0)   [ENU]
    Every tick (cmd_hz):
      1. bearing to the goal relative to the CURRENT heading is sent to the
         VFH node as /vfh/target_angle (VFH convention: 0 = ahead, + = right);
         the barometric altitude above take-off goes out on /vfh/agl_m
         (10 Hz) for the perception node's ground filter
      2. the latest /vfh/movement_direction (state, steer_deg, front_min_m)
         decides:
           move/avoid : desired heading = current heading - steer_deg
                        (VFH + = right = clockwise = NEGATIVE ENU yaw)
           stop       : no free valley -> XY setpoint FROZEN, yaw searches
                        toward avoid_toward at half rate
           goal off the nose by more than goal_fov_half_deg (80: almost
                        beside/behind): XY frozen, turn toward the goal first
           no progress toward the goal for blocked_timeout_s (blocked,
                        oscillating, stuck) -> leg ends, LAND HERE
           stale      : no fresh VFH message for vfh_stale_s -> XY frozen;
                        longer than vfh_stale_abort_s -> leg ends, LAND HERE
           front_min_m < stop_dist -> hard brake: XY frozen (yaw still turns)
      3. the yaw setpoint slews toward the desired heading at
         yaw_rate_max_deg; the XY setpoint advances along the YAW SETPOINT
         at forward_speed ONLY while (a) the drone's heading is within
         move_heading_tol_deg of it (the camera must look where we go),
         (b) the drone is within max_lead of the setpoint, and (c) the
         goal is not behind the heading. So the drone never flies where
         the camera is not looking.
      4. done when the drone is within goal_tol of the goal, or when
         mission_timeout_s runs out (LAND HERE). The anchor for HOLD and
         LANDING is the last setpoint.
    Safety during the leg (all inherited unless stated):
      - max_pos_error = TRACKING error (drone vs moving setpoint), as in
        forward_move; max_alt_error on the barometric altitude; vz limit
      - geofence (new): distance from the launch point >
        goal_dist + geofence_margin -> sanity abort (AUTO.LAND)
      - Kill / RC override / link loss / unexpected disarm -> inherited
        terminal handlers; barometer stream lost -> AUTO.LAND
    The camera is FIXED, FRONT-FACING (Orbbec Gemini 2). There is no
    perception to the sides or rear, and VFH is 2-D: nothing above or
    below the camera band is seen. Fly at a height with clear sky above.
"""
import json
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, String

from forward_move.forward_move_node import ForwardMoveNode
from takeoff_land_barometer.takeoff_land_baro_node import TakeoffLandBaroMixin

_MAX_GOAL_DIST = 30.0   # m
_TAG = "[VFH]"


def _wrap(a: float) -> float:
    """Wrap an angle (rad) to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class VFHFlightBaroNode(TakeoffLandBaroMixin, ForwardMoveNode):

    def __init__(self):
        super().__init__()          # ForwardMoveNode -> TakeoffLandNode
        self._baro_init("[BARO]")   # barometric altitude loop (mixin)

        p = self.declare_parameter
        p("goal_dist",            3.0)    # m ahead of the launch point
        p("goal_tol",             0.5)    # m; drone within this -> goal reached
        p("yaw_rate_max_deg",     30.0)   # deg/s yaw setpoint slew
        p("move_heading_tol_deg", 25.0)   # move only when heading is this close
        p("max_lead",             0.6)    # m; setpoint may lead the drone by this
        p("stop_dist",            1.2)    # m; front_min below this -> hard brake
        p("vfh_stale_s",          1.0)    # s; no VFH message -> freeze XY
        p("vfh_stale_abort_s",    5.0)    # s; no VFH message -> land here
        p("blocked_timeout_s",    15.0)   # s without progress to the goal -> land here
        p("goal_fov_half_deg",    80.0)   # goal farther off the nose than this: turn to it first
        p("mission_timeout_s",    120.0)  # s for the whole leg -> land here
        p("geofence_margin",      2.0)    # m beyond goal_dist from launch -> AUTO.LAND
        p("require_vfh",          True)   # pre-flight: VFH stream must be alive
        p("vfh_min_rate_hz",      2.0)    # pre-flight: VFH message rate
        p("vfh_wait_timeout_s",   60.0)   # pre-flight: how long to wait for it

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._goal_dist      = float(g("goal_dist"))
        self._goal_tol       = float(g("goal_tol"))
        self._yaw_rate       = math.radians(float(g("yaw_rate_max_deg")))
        self._move_head_tol  = math.radians(float(g("move_heading_tol_deg")))
        self._max_lead       = float(g("max_lead"))
        self._stop_dist      = float(g("stop_dist"))
        self._vfh_stale_s    = float(g("vfh_stale_s"))
        self._vfh_abort_s    = float(g("vfh_stale_abort_s"))
        self._blocked_to     = float(g("blocked_timeout_s"))
        self._goal_fov_half  = math.radians(float(g("goal_fov_half_deg")))
        self._mission_to     = float(g("mission_timeout_s"))
        self._geofence_m     = float(g("geofence_margin"))
        self._require_vfh    = bool(g("require_vfh"))
        self._vfh_min_rate   = float(g("vfh_min_rate_hz"))
        self._vfh_wait_to    = float(g("vfh_wait_timeout_s"))

        # forward_move's end point / banners use _fwd_dist: the goal IS the
        # forward_move end point, only the path there may bend.
        self._fwd_dist = self._goal_dist

        self._vfh_lock   = threading.Lock()
        self._vfh_msg    = None      # last decoded /vfh/movement_direction
        self._vfh_t      = None      # monotonic receive time
        self._vfh_times  = []        # receive times (rate estimate)
        self._vfh_n      = 0
        self._leg_result = ""        # why the leg ended (for the logs)
        self._launch_yaw = 0.0

        self.create_subscription(String, "/vfh/movement_direction", self._cb_vfh, 10)
        self._pub_target = self.create_publisher(Float32, "/vfh/target_angle", 10)
        # barometric altitude above the take-off point -> perception ground
        # filter (0 until the ground reference exists)
        self._pub_agl = self.create_publisher(Float32, "/vfh/agl_m", 10)
        self.create_timer(0.1, self._publish_agl)

        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"{_TAG} VFH obstacle-avoidance leg: goal {self._goal_dist:.1f} m ahead "
            f"(tol {self._goal_tol:.1f} m) at {self._fwd_speed:.2f} m/s, yaw slew "
            f"{math.degrees(self._yaw_rate):.0f} deg/s, hard brake < {self._stop_dist:.1f} m, "
            f"blocked > {self._blocked_to:.0f} s or no VFH > {self._vfh_abort_s:.0f} s "
            "-> land here")
        self.get_logger().info(
            f"{_TAG} geofence: > {self._goal_dist + self._geofence_m:.1f} m from launch "
            f"-> AUTO.LAND. Camera is front-facing only; VFH is 2-D (no view up/down).")
        self.get_logger().info("=" * 62)

    # ── VFH stream ────────────────────────────────────────────────────────
    def _cb_vfh(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        now = time.monotonic()
        with self._vfh_lock:
            self._vfh_msg = d
            self._vfh_t   = now
            self._vfh_n  += 1
            self._vfh_times.append(now)
            if len(self._vfh_times) > 100:
                del self._vfh_times[:-100]

    def _publish_agl(self):
        try:
            agl = self._baro_alt() if self._baro.ready else 0.0
        except Exception:
            agl = 0.0
        self._pub_agl.publish(Float32(data=float(agl)))

    def _vfh_latest(self):
        """(dict or None, age in s or inf)."""
        with self._vfh_lock:
            if self._vfh_msg is None:
                return None, float("inf")
            return self._vfh_msg, time.monotonic() - self._vfh_t

    def _vfh_rate_hz(self, window_s=3.0):
        with self._vfh_lock:
            now = time.monotonic()
            n = sum(1 for t in self._vfh_times if now - t <= window_s)
        return n / window_s

    def _wait_vfh_stream(self) -> bool:
        """Pre-flight: the perception node must be alive and processing
        frames before the propellers spin."""
        if not self._require_vfh:
            self.get_logger().warn(
                f"{_TAG} require_vfh:=false — NOT checking the VFH stream before "
                "arming. The leg will hold position if it never arrives.")
            return True
        t0 = time.time(); last = 0.0
        while rclpy.ok() and time.time() - t0 < self._vfh_wait_to:
            rate = self._vfh_rate_hz()
            msg, age = self._vfh_latest()
            if rate >= self._vfh_min_rate and age < self._vfh_stale_s and msg is not None:
                self.get_logger().info(
                    f"{_TAG} VFH stream OK: {rate:.1f} Hz, state={msg.get('state')} "
                    f"hfov={msg.get('hfov_deg')} deg, {msg.get('proc_ms')} ms/frame, "
                    f"front_min={msg.get('front_min_m')} m")
                return True
            if time.time() - last >= 3.0:
                last = time.time()
                self.get_logger().warn(
                    f"{_TAG} waiting for /vfh/movement_direction "
                    f"({rate:.1f} Hz, need >= {self._vfh_min_rate:.0f} Hz; "
                    f"{self._vfh_wait_to - (time.time() - t0):.0f} s left) — "
                    "camera / depth bridge / vfh_avoidance_node running?")
            time.sleep(0.2)
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            f"{_TAG} NO VFH PERCEPTION STREAM — REFUSING TO ARM. Check the Orbbec "
            "driver (/camera/depth/image_raw), gemini2_depth_bridge_node "
            "(/realsense/depth/float32) and vfh_avoidance_node.")
        self.get_logger().error("=" * 62)
        return False

    # ── pre-flight hook: ground reference wait is where we also check VFH ──
    def _wait_ekf_stable(self, *args, **kwargs):
        if not self._wait_vfh_stream():
            return None       # run_sequence aborts before READY / ARMING
        return super()._wait_ekf_stable(*args, **kwargs)

    # ── forward_move parameter check (goal may be longer than 10 m) ───────
    def _forward_params_problem(self):
        if not 0.0 < self._goal_dist <= _MAX_GOAL_DIST:
            return f"goal_dist={self._goal_dist} m (need 0 < d <= {_MAX_GOAL_DIST} m)"
        if self._goal_tol <= 0.0 or self._goal_tol >= self._goal_dist:
            return f"goal_tol={self._goal_tol} m (need 0 < tol < goal_dist)"
        if self._stop_dist < 0.3:
            return f"stop_dist={self._stop_dist} m (need >= 0.3)"
        if self._max_lead <= 0.0 or self._max_lead > self._max_pos_error:
            return (f"max_lead={self._max_lead} m (need 0 < lead <= "
                    f"max_pos_error {self._max_pos_error})")
        saved = self._fwd_dist
        self._fwd_dist = min(self._goal_dist, 10.0)   # let the parent check speed/hold
        try:
            return super()._forward_params_problem()
        finally:
            self._fwd_dist = saved

    # ── cosmetic: forward_move's banner texts describe a straight leg ─────
    def _announce_phase(self, name, detail="", warn=False):
        if name == "FORWARD":
            detail = (f"VFH-guided flight to the goal {self._goal_dist:.1f} m ahead "
                      f"at {self._fwd_speed:.2f} m/s (steering around obstacles; "
                      f"stop < {self._stop_dist:.1f} m). Abort limits: tracking err "
                      f"{self._max_pos_error:.1f} m, alt err {self._max_alt_error:.1f} m, "
                      f"geofence {self._goal_dist + self._geofence_m:.1f} m")
        elif name == "DONE":
            detail = f"MISSION COMPLETE — {self._leg_result or 'flew the VFH leg'}, landed & disarmed"
        return super()._announce_phase(name, detail, warn)

    def _phase_note(self, text, warn=False):
        if "NO OBSTACLE AVOIDANCE" in text:
            text = ("VFH obstacle avoidance ACTIVE (front camera only, 2-D). "
                    "Sky above the cruise height must be clear; nothing is seen "
                    "to the sides or behind.")
        return super()._phase_note(text, warn)

    # ── THE VFH LEG (replaces forward_move._fly_forward) ──────────────────
    def _fly_forward(self, z: float) -> bool:
        return self._fly_vfh(z)

    def _fly_vfh(self, z: float) -> bool:
        """Returns True when the leg is over and HOLD/LANDING should happen
        at the current anchor (goal reached, or a recoverable stop: blocked,
        perception lost, timeout). Returns False when the mission must stop
        (abort flag set — the caller dispatches; sanity abort already sent
        AUTO.LAND; disarm; ROS shutting down)."""
        self._launch_yaw = float(self._home_yaw)
        x0, y0 = self._launch_x, self._launch_y
        gx = x0 + self._goal_dist * math.cos(self._launch_yaw)
        gy = y0 + self._goal_dist * math.sin(self._launch_yaw)
        fence = self._goal_dist + self._geofence_m

        sx, sy   = x0, y0                    # XY setpoint (the anchor)
        yaw_sp   = _wrap(float(self._yaw))   # yaw setpoint starts at the real heading
        dt       = 1.0 / self._cmd_hz
        t0       = time.time()
        t0_mono  = time.monotonic()
        t_target_pub = 0.0
        mode = "start"
        # progress watchdog: the closest we have been to the goal, and when
        best_d_goal = float("inf")
        t_progress  = time.time()
        self.get_logger().info(
            f"{_TAG} goal=({gx:.2f},{gy:.2f}) from launch=({x0:.2f},{y0:.2f}) "
            f"yaw0={math.degrees(self._launch_yaw):.1f} deg ENU")

        while rclpy.ok():
            if (self._kill_latched or self._disarm_abort
                    or self._rc_override or self._link_lost):
                return False
            # anchor = the setpoint -> max_pos_error is a tracking limit
            self._home_x, self._home_y = sx, sy
            if not self._sanity_check(z, check_alt=True):
                return False
            if not self._armed:
                return False

            px, py = float(self._pos[0]), float(self._pos[1])
            yaw    = float(self._yaw)
            d_goal = math.hypot(gx - px, gy - py)
            d_home = math.hypot(px - x0, py - y0)
            if d_goal <= self._goal_tol:
                self._leg_result = f"goal reached ({d_goal:.2f} m from it)"
                self._phase_note(f"{self._leg_result}; holding here")
                return True
            if d_home > fence:
                self._sanity_abort(
                    f"VFH GEOFENCE: {d_home:.2f} m from launch (limit {fence:.1f} m)")
                return False
            if time.time() - t0 > self._mission_to:
                self._leg_result = (f"mission timeout {self._mission_to:.0f} s, "
                                    f"{d_goal:.1f} m short of the goal")
                self._phase_note(f"{self._leg_result} — landing here", warn=True)
                return True
            # no progress toward the goal (blocked, oscillating, or stuck
            # turning) for blocked_timeout_s -> give up and land here
            if d_goal < best_d_goal - 0.05:
                best_d_goal = d_goal
                t_progress  = time.time()
            elif time.time() - t_progress > self._blocked_to:
                self._leg_result = (f"no progress for {self._blocked_to:.0f} s "
                                    f"(blocked), {d_goal:.1f} m short of the goal")
                self._phase_note(f"{self._leg_result} — landing here", warn=True)
                return True

            # 1. goal bearing relative to the CURRENT heading -> VFH target
            bearing = math.atan2(gy - py, gx - px)
            rel     = _wrap(bearing - yaw)              # + = left (CCW)
            vfh_target = -math.degrees(rel)             # VFH: + = right
            if time.time() - t_target_pub >= 0.1:
                t_target_pub = time.time()
                self._pub_target.publish(Float32(data=float(vfh_target % 360.0)))

            # 2. what does the perception say?
            vfh, age = self._vfh_latest()
            if vfh is None:                 # never received: age since the leg began
                age = time.monotonic() - t0_mono
            advance  = False
            yaw_des  = yaw_sp
            rate     = self._yaw_rate
            if vfh is None or age > self._vfh_stale_s:
                mode = "NO-VFH"
                if age > self._vfh_abort_s:
                    self._leg_result = (f"VFH perception lost for {age:.1f} s, "
                                        f"{d_goal:.1f} m short of the goal")
                    self._phase_note(f"{self._leg_result} — landing here", warn=True)
                    return True
            else:
                state = str(vfh.get("state", "stop"))
                steer = float(vfh.get("steer_deg") or 0.0)
                fmin  = vfh.get("front_min_m")
                fmin  = None if fmin is None else float(fmin)
                if state == "stop":
                    mode = "BLOCKED"
                    # search: turn toward the emptier side, slowly, but never
                    # more than 120 deg away from the goal bearing
                    side = 1.0 if str(vfh.get("avoid_toward")) == "left" else -1.0
                    yaw_des = yaw + side * math.radians(20.0)
                    if abs(_wrap(yaw_des - bearing)) > math.radians(120.0):
                        yaw_des = yaw_sp
                    rate = self._yaw_rate * 0.5
                elif abs(rel) > self._goal_fov_half:
                    # The goal is almost beside or behind the nose: moving
                    # along the heading would not bring us closer and VFH
                    # (target clamped into the FOV) cannot point at it. Turn
                    # toward the goal first, XY frozen; VFH takes over again
                    # inside the cone (and 'stop' above if a wall is there —
                    # the progress watchdog ends an oscillation). Inside the
                    # cone VFH's own clamped target already steers toward
                    # the goal side, so the cone must stay wide (80 deg).
                    mode = "TURN-TO-GOAL"
                    yaw_des = yaw + max(-self._goal_fov_half,
                                        min(self._goal_fov_half, rel))
                else:
                    yaw_des = yaw - math.radians(steer)   # VFH + right = CW = -ENU
                    if fmin is not None and fmin < self._stop_dist:
                        mode = f"BRAKE {fmin:.2f}m"
                    else:
                        mode = state.upper()
                        advance = True

            # 3. slew yaw, advance XY along the yaw setpoint
            yaw_sp = _wrap(yaw_sp + max(-rate * dt, min(rate * dt, _wrap(yaw_des - yaw_sp))))
            head_err = abs(_wrap(yaw_sp - yaw))
            lead     = math.hypot(sx - px, sy - py)
            if advance and head_err <= self._move_head_tol and lead <= self._max_lead:
                cx, cy = math.cos(yaw_sp), math.sin(yaw_sp)
                along  = (gx - sx) * cx + (gy - sy) * cy      # goal ahead of the sp?
                step   = min(self._fwd_speed * dt, max(0.0, along))
                sx += step * cx
                sy += step * cy
            elif advance:
                mode += " (turning)" if head_err > self._move_head_tol else " (lagging)"

            # HOLD / LANDING (inherited) keep the heading the leg ended with
            self._home_yaw = yaw_sp
            self._set_sp(sx, sy, z, yaw_sp)
            steer_log = 0.0 if vfh is None else float(vfh.get("steer_deg") or 0.0)
            fm = None if vfh is None else vfh.get("front_min_m")
            front_log = "-" if fm is None else f"{float(fm):.2f}m"
            self.get_logger().info(
                f"{self._c('white')}[FORWARD]{self._c('reset')} {mode:<14s} "
                f"goal {d_goal:4.1f} m at {math.degrees(rel):+5.0f} deg | "
                f"steer {steer_log:+5.1f} | "
                f"yaw {math.degrees(yaw):+6.1f} -> sp {math.degrees(yaw_sp):+6.1f} | "
                f"front {front_log} | lead {lead:.2f} m | "
                f"vfh {0.0 if age == float('inf') else age:.1f}s",
                throttle_duration_sec=1.0)
            time.sleep(dt)
        return False


def main(args=None):
    argv = list(sys.argv if args is None else args)
    if not any(a.startswith("__node:=") or a.startswith("__name:=") for a in argv):
        argv += ["--ros-args", "-r", "__node:=vfh_flight_baro_node"]
    rclpy.init(args=argv)
    node = VFHFlightBaroNode()

    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(f"{_TAG} Stopped by user (Ctrl-C).")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
