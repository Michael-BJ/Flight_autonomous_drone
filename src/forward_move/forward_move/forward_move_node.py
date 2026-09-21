#!/usr/bin/env python3
"""
forward_move_node.py
==================================================================
PURPOSE: TAKE OFF -> HOVER -> FORWARD (straight line along the launch
heading) -> HOLD -> LANDING at the end point, via OFFBOARD mode.

TAKEOFF & LANDING ARE takeoff_land's OWN CODE, NOT A COPY:
    ForwardMoveNode subclasses takeoff_land.takeoff_land_node.TakeoffLandNode.
    Every takeoff_land parameter, pre-flight gate (COM_RC_OVERRIDE, GPS,
    EKF ground_z, RC mode switch, kill switch), the ARM/OFFBOARD handshake,
    _wait_altitude, _controlled_descent, the AUTO.LAND hand-off and every
    abort path (kill switch, unexpected disarm, RC override, link loss,
    sanity) are the inherited methods. A fix made in takeoff_land applies
    here too once takeoff_land is rebuilt.

    The one thing that IS copied is the ORDER of those calls, run_sequence():
    takeoff_land's version has no hook between HOVER and LANDING. When
    takeoff_land_node.run_sequence() changes, bring the same change here.
    Every line that differs from it is tagged "FORWARD".

FORWARD PHASE:
    - Direction = the heading (yaw) locked with home, i.e. where the nose
      points when the mission starts. Yaw is held constant the whole flight.
    - The XY setpoint slides from home to
      home + forward_distance * (cos yaw, sin yaw)   [ENU, like the setpoints]
      at forward_speed m/s. Altitude stays at takeoff_z.
    - Sanity: takeoff_land's _sanity_check() measures the horizontal error
      from self._home_x/_home_y. During FORWARD that anchor moves with the
      setpoint, so max_pos_error becomes a TRACKING-error limit (actual vs
      commanded point). Altitude and vz limits are unchanged. Exceeding any
      of them -> AUTO.LAND, exactly as in takeoff_land. The "drift=" field of
      the status line shows the same tracking error.
    - After FORWARD the anchor stays at the end point, so HOLD and LANDING
      (controlled descent -> AUTO.LAND) happen there, and the drift limit is
      measured from the end point.
    - NO OBSTACLE AVOIDANCE. The depth camera is not used. The pilot must
      check that the path is clear before launching.
    - If the takeoff climb never became stable (takeoff_land then continues
      "carefully"), FORWARD is skipped and the drone lands where it took off.

FSM FLOW:
    IDLE -> (connect) -> (GPS gate) -> (EKF stable -> ground_z)
         -> ARM -> OFFBOARD -> TAKEOFF -> HOVER -> FORWARD -> HOLD
         -> LANDING (controlled descent -> AUTO.LAND -> auto-disarm) -> DONE
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from takeoff_land.takeoff_land_node import TakeoffLandNode


# Hard limits on the forward leg, checked before anything arms. Position is
# GPS-only and nothing looks for obstacles, so the leg stays short and slow.
_MAX_FORWARD_DISTANCE = 10.0   # m
_MIN_FORWARD_SPEED    = 0.05   # m/s
_MAX_FORWARD_SPEED    = 1.0    # m/s

# Same monitoring view as takeoff_land, plus FORWARD. HOLD is reported as a
# note inside FORWARD, not as its own phase.
_PHASE_ORDER = ["PREFLIGHT", "READY", "ARMING", "TAKEOFF", "HOVER",
                "FORWARD", "LANDING"]
_PHASE_COLOR = {
    "PREFLIGHT": "cyan",  "READY":   "green",  "ARMING":  "yellow",
    "TAKEOFF":   "yellow", "HOVER":  "blue",   "FORWARD": "white",
    "LANDING":   "cyan",  "DONE":    "green",  "ABORT":   "red",
}


class ForwardMoveNode(TakeoffLandNode):

    STATE_FORWARD = "FORWARD"

    def __init__(self):
        super().__init__()

        # ── Forward leg parameters (everything else is takeoff_land's) ───────
        self.declare_parameter("forward_distance",  2.0)   # m along launch heading
        self.declare_parameter("forward_speed",     0.3)   # m/s setpoint speed
        self.declare_parameter("forward_hold_time", 3.0)   # s at the end point

        self._fwd_dist  = float(self.get_parameter("forward_distance").value)
        self._fwd_speed = float(self.get_parameter("forward_speed").value)
        self._fwd_hold  = float(self.get_parameter("forward_hold_time").value)

        # Launch point, kept apart from self._home_x/_home_y, which become the
        # moving sanity anchor during FORWARD.
        self._launch_x = 0.0
        self._launch_y = 0.0

        self.get_logger().info(
            f"[FWD] forward leg: {self._fwd_dist:.2f} m at "
            f"{self._fwd_speed:.2f} m/s, hold {self._fwd_hold:.1f} s, "
            "then land at the end point. NO obstacle avoidance.")

    # ── Monitoring: takeoff_land's banner with FORWARD in the phase list ─────
    def _announce_phase(self, name: str, detail: str = "", warn: bool = False):
        """Unmissable banner on every mission-phase transition."""
        prev = self._phase_name
        self._phase_name = name
        self._t_phase    = time.time()
        if name in _PHASE_ORDER:
            self._phase_idx = _PHASE_ORDER.index(name) + 1
        col  = self._c(_PHASE_COLOR.get(name, "white"))
        bold = self._c("bold")
        rst  = self._c("reset")
        step = (f"[{self._phase_idx}/{len(_PHASE_ORDER)}] "
                if name in _PHASE_ORDER else "")
        self._emit(f"{col}{'=' * 62}{rst}", warn)
        self._emit(f"{col}{bold}  >>> {step}{name}{rst}"
                   f"{self._c('dim')}   (from {prev})   {self._elapsed()}{rst}",
                   warn)
        if detail:
            self._emit(f"{col}      {detail}{rst}", warn)
        self._emit(f"{col}{'=' * 62}{rst}", warn)

    def _phase_note(self, text: str, warn: bool = False):
        """One-line sub-status inside the current phase (no banner)."""
        col = self._c(_PHASE_COLOR.get(self._phase_name, "white"))
        rst = self._c("reset")
        self._emit(f"{col}[{self._phase_name}]{rst} {text}", warn)

    # ── FORWARD ──────────────────────────────────────────────────────────────
    def _forward_params_problem(self):
        """None when the forward-leg parameters are usable, else the reason."""
        if not 0.0 < self._fwd_dist <= _MAX_FORWARD_DISTANCE:
            return (f"forward_distance={self._fwd_dist} m "
                    f"(need 0 < d <= {_MAX_FORWARD_DISTANCE} m)")
        if not _MIN_FORWARD_SPEED <= self._fwd_speed <= _MAX_FORWARD_SPEED:
            return (f"forward_speed={self._fwd_speed} m/s (need "
                    f"{_MIN_FORWARD_SPEED}-{_MAX_FORWARD_SPEED} m/s)")
        if self._fwd_hold < 0.0:
            return f"forward_hold_time={self._fwd_hold} s (need >= 0)"
        return None

    def _fly_forward(self, z: float) -> bool:
        """Slide the XY setpoint from the launch point to the end point.

        The setpoint position is computed from elapsed time, so it moves at
        forward_speed regardless of loop jitter. Returns True once the
        setpoint sits on the end point, False when the mission has to stop
        (an abort flag is set, a sanity abort already handed off to AUTO.LAND,
        or ROS is shutting down) — the caller dispatches the abort."""
        c, s = math.cos(self._home_yaw), math.sin(self._home_yaw)
        x0, y0 = self._launch_x, self._launch_y
        dt = 1.0 / self._cmd_hz
        t0 = time.time()
        while rclpy.ok():
            if (self._kill_latched or self._disarm_abort
                    or self._rc_override or self._link_lost):
                return False
            d = min(self._fwd_dist, self._fwd_speed * (time.time() - t0))
            sx, sy = x0 + d * c, y0 + d * s
            # Moving anchor: _sanity_check's "drift from home" is now the
            # tracking error between the drone and the commanded point.
            self._home_x, self._home_y = sx, sy
            if not self._sanity_check(z, check_alt=True):
                return False
            self._set_sp(sx, sy, z, self._home_yaw)
            if not self._armed:
                return False
            px = float(self._pos[0]) - x0
            py = float(self._pos[1]) - y0
            self.get_logger().info(
                f"{self._c('white')}[FORWARD]{self._c('reset')} "
                f"setpoint {d:4.2f}/{self._fwd_dist:.2f} m | drone "
                f"{px * c + py * s:+5.2f} m ahead, "
                f"{-px * s + py * c:+5.2f} m left of the line",
                throttle_duration_sec=1.0)
            if d >= self._fwd_dist:
                return True
            time.sleep(dt)
        return False

    def _hold_at_anchor(self, z: float, duration: float) -> bool:
        """Hold the current anchor (self._home_x/_home_y) for duration s.
        Same checks as takeoff_land's HOVER loop. Returns False when the
        mission has to stop; the caller dispatches the abort."""
        t_h = time.time()
        while rclpy.ok() and time.time() - t_h < duration:
            if (self._kill_latched or self._disarm_abort
                    or self._rc_override or self._link_lost):
                return False
            if not self._sanity_check(z, check_alt=True):
                return False
            self._set_sp(self._home_x, self._home_y, z, self._home_yaw)
            if not self._armed:
                return False
            self.get_logger().info(
                f"{self._c('white')}[HOLD]{self._c('reset')} "
                f"landing in {duration - (time.time() - t_h):4.1f} s...",
                throttle_duration_sec=2.0)
            time.sleep(0.1)
        return rclpy.ok()

    def _dispatch_abort(self):
        """Route a stopped FORWARD/HOLD to takeoff_land's abort handlers.
        Returns True when one of them ran."""
        if self._kill_latched:
            self._kill_abort()
        elif self._disarm_abort:
            self._disarm_stop()
        elif self._rc_override:
            self._rc_override_abort()
        elif self._link_lost:
            self._link_lost_abort()
        else:
            return False
        return True

    # ── Mission sequence (separate thread) ────────────────────────────────────
    # COPIED FROM takeoff_land_node.run_sequence() on 2026-09-10. Only lines
    # tagged FORWARD differ. Keep in step with the original.
    def run_sequence(self):
        self._t_run_start = time.time()
        self._announce_phase(
            "PREFLIGHT",
            "FCU -> COM_RC_OVERRIDE -> local pose -> GPS gate -> ground_z")

        # FORWARD: refuse bad forward-leg parameters before anything else.
        problem = self._forward_params_problem()
        if problem is not None:
            self.get_logger().error(f"[FWD] Invalid parameter: {problem} — aborting.")
            return self._safe_shutdown()

        # 1. Wait for connection
        self.get_logger().info("[T/L] Waiting for MAVROS connection (via /px4/state)...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > self._conn_timeout:
                self.get_logger().error("[T/L] Connection timeout — aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)
        self.get_logger().info("[T/L] FCU connected.")

        if not self._check_rc_override_param():
            self.get_logger().error(
                "[T/L] Aborting — RC override safety param not confirmed.")
            return self._safe_shutdown()

        # 2. Wait for valid local position
        self.get_logger().info("[T/L] Waiting for local position (from /px4/sensors)...")
        t0 = time.time()
        while rclpy.ok() and not self._have_pose:
            if time.time() - t0 > 30.0:
                self.get_logger().error(
                    "[T/L] No local position. PX4 needs GPS/VIO/flow "
                    "for OFFBOARD position control. Aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)

        # 2b. GPS quality gate
        if not self._wait_gps_quality():
            self.get_logger().error("[T/L] Aborting — GPS not safe to fly on.")
            return self._safe_shutdown()

        # 3. Ground Z
        if self._use_ekf_stable:
            ground_z = self._wait_ekf_stable()
            if ground_z is None:
                self.get_logger().error(
                    "[T/L] Aborting — ground_z could not be established.")
                return self._safe_shutdown()
            self._ground_z = ground_z
        else:
            self._ground_z = float(self._pos[2])
            self.get_logger().info("[T/L] EKF stable wait skipped.")

        # 4. Lock home position
        self._home_x   = float(self._pos[0])
        self._home_y   = float(self._pos[1])
        self._home_yaw = float(self._yaw)
        self._launch_x, self._launch_y = self._home_x, self._home_y   # FORWARD
        takeoff_z = self._ground_z + self._alt
        self.get_logger().info(
            f"[T/L] home=({self._home_x:.2f},{self._home_y:.2f}) "
            f"ground_z={self._ground_z:.2f}m -> takeoff_z={takeoff_z:.2f}m")

        # FORWARD: end point along the locked heading (ENU yaw, rad).
        end_x = self._launch_x + self._fwd_dist * math.cos(self._home_yaw)
        end_y = self._launch_y + self._fwd_dist * math.sin(self._home_yaw)
        yaw_deg = math.degrees(self._home_yaw)
        compass = (90.0 - yaw_deg) % 360.0   # ENU yaw -> compass heading
        self.get_logger().info(
            f"[FWD] heading yaw={yaw_deg:.1f} deg ENU (compass ~{compass:.0f} deg) "
            f"-> end point=({end_x:.2f},{end_y:.2f})")

        self._set_sp(self._home_x, self._home_y, self._ground_z, self._home_yaw)

        # 4b. READY
        self._announce_phase(
            "READY",
            f"DRONE READY TO LAUNCH — GPS ok, ground_z={self._ground_z:.2f} m, "
            f"home=({self._home_x:.2f},{self._home_y:.2f}), "
            f"takeoff_z={takeoff_z:.2f} m (physical ~{self._alt:.1f} m)")
        self._phase_note(                                               # FORWARD
            f"hover {self._hover_time:.0f} s, fly {self._fwd_dist:.1f} m "
            f"FORWARD (compass ~{compass:.0f} deg) at {self._fwd_speed:.2f} m/s, "
            f"hold {self._fwd_hold:.0f} s, then descend at "
            f"{self._descent_speed:.2f} m/s to {self._land_handoff:.2f} m "
            f"and hand off to AUTO.LAND at the end point")
        self._phase_note(                                               # FORWARD
            f"NO OBSTACLE AVOIDANCE — the {self._fwd_dist:.1f} m ahead of the "
            "nose must be clear.", warn=True)
        self._phase_note(
            "PILOT: hold the RC with the mode switch ready. Next step ARMS "
            "the drone.", warn=True)

        # 5b. RC mode switch in the OFFBOARD slot
        if not self._check_rc_offboard_position():
            return self._safe_shutdown()

        # 5c. Kill switch released; from here on engaging it is terminal
        if not self._check_kill_switch_released():
            return self._safe_shutdown()
        self._kill_watch = True

        # 5. Warm-up setpoint stream
        self._announce_phase(
            "ARMING",
            "PROPELLERS WILL SPIN — streaming setpoints, then ARM + OFFBOARD",
            warn=True)
        self._phase_note("warming up setpoint stream (~2s)...")
        time.sleep(2.0)
        if self._kill_latched:
            return self._kill_abort()

        # 6. ARM
        self._phase_note("sending ARM...")
        t0 = time.time(); armed = False
        while rclpy.ok() and time.time() - t0 < self._arm_timeout:
            if self._kill_latched:
                return self._kill_abort()
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True
                    break
            time.sleep(1.5)
        if self._kill_latched:
            return self._kill_abort()
        if not armed:
            self.get_logger().error("[T/L] Arming failed — aborting.")
            return self._safe_shutdown()

        # 7. OFFBOARD
        if not self._set_mode("OFFBOARD"):
            self.get_logger().error("[T/L] Failed to send OFFBOARD — disarming & aborting.")
            self._arm(False)
            return self._safe_shutdown()
        t0 = time.time()
        while rclpy.ok() and self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            if self._kill_latched:
                return self._kill_abort()
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._kill_latched:
            return self._kill_abort()
        if self._mode != "OFFBOARD":
            self.get_logger().error("[T/L] PX4 did not enter OFFBOARD — disarming & aborting.")
            self._arm(False)
            return self._safe_shutdown()

        # RC override detection is active from here until AUTO.LAND handoff
        self._in_offboard_mission = True
        if not self._armed:
            return self._disarm_stop()

        # 8. TAKEOFF
        self._mission = self.STATE_TAKEOFF
        self._announce_phase(
            "TAKEOFF",
            f"climbing to z={takeoff_z:.2f} m (physical ~{self._alt:.1f} m), "
            f"XY pinned to home. Abort limits: drift "
            f"{self._max_pos_error:.1f} m, alt err {self._max_alt_error:.1f} m")
        self._set_sp(self._home_x, self._home_y, takeoff_z, self._home_yaw)
        stable = self._wait_altitude(takeoff_z, tol=0.15, timeout=30.0)
        if self._kill_latched:
            return self._kill_abort()
        if self._disarm_abort:
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()
        if self._link_lost:
            return self._link_lost_abort()
        if not rclpy.ok():
            return
        if not stable:
            self._phase_note(
                "altitude never became stable within 30 s — continuing "
                "carefully (mission proceeds by design)", warn=True)
        else:
            self._phase_note(
                f"reached target altitude, alt="
                f"{float(self._pos[2]) - self._ground_z:.2f} m")

        # 9. HOVER
        self._mission = self.STATE_HOVER
        self._announce_phase(
            "HOVER",
            f"holding home for {self._hover_time:.0f} s at "
            f"z={takeoff_z:.2f} m before moving forward")               # FORWARD
        t_h = time.time()
        while rclpy.ok() and time.time() - t_h < self._hover_time:
            if self._kill_latched:
                return self._kill_abort()
            if self._disarm_abort:
                return self._disarm_stop()
            if self._rc_override:
                return self._rc_override_abort()
            if self._link_lost:
                return self._link_lost_abort()
            if not self._sanity_check(takeoff_z, check_alt=True):
                return
            self._set_sp(self._home_x, self._home_y, takeoff_z, self._home_yaw)
            self.get_logger().info(
                f"{self._c('blue')}[HOVER]{self._c('reset')} "
                f"moving forward in {self._hover_time - (time.time() - t_h):4.1f} s...",
                throttle_duration_sec=2.0)                              # FORWARD
            if not self._armed:
                self._announce_phase(
                    "ABORT", "unexpected DISARM during hover", warn=True)
                return self._safe_shutdown()
            time.sleep(0.1)

        # FORWARD: 9b. fly the forward leg, then hold at the end point.
        if not rclpy.ok():
            return
        if not stable:
            self._phase_note(
                "takeoff climb was not stable — SKIPPING FORWARD, landing "
                "where the drone took off", warn=True)
        else:
            self._mission = self.STATE_FORWARD
            self._announce_phase(
                "FORWARD",
                f"flying {self._fwd_dist:.1f} m straight ahead (compass "
                f"~{compass:.0f} deg) at {self._fwd_speed:.2f} m/s, "
                f"z={takeoff_z:.2f} m. Abort limits: tracking err "
                f"{self._max_pos_error:.1f} m, alt err {self._max_alt_error:.1f} m")
            reached = self._fly_forward(takeoff_z)
            if self._dispatch_abort():
                return
            if not reached:
                if rclpy.ok() and not self._armed:
                    self._announce_phase(
                        "ABORT", "unexpected DISARM during forward", warn=True)
                    return self._safe_shutdown()
                return   # sanity abort already sent AUTO.LAND, or shutdown
            self._phase_note(
                f"setpoint at the end point — holding {self._fwd_hold:.0f} s")
            held = self._hold_at_anchor(takeoff_z, self._fwd_hold)
            if self._dispatch_abort():
                return
            if not held:
                if rclpy.ok() and not self._armed:
                    self._announce_phase(
                        "ABORT", "unexpected DISARM during hold", warn=True)
                    return self._safe_shutdown()
                return
            ex = float(self._pos[0]) - end_x
            ey = float(self._pos[1]) - end_y
            self._phase_note(
                f"end point reached, error {math.hypot(ex, ey):.2f} m; "
                "landing here")

        # 10. LANDING
        self._mission = self.STATE_LANDING
        self._announce_phase(
            "LANDING",
            f"controlled descent at {self._descent_speed:.2f} m/s to "
            f"{self._land_handoff:.2f} m above ground, then AUTO.LAND")
        self._controlled_descent(takeoff_z)
        if self._kill_latched:
            return self._kill_abort()
        if self._disarm_abort:
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()
        if self._link_lost:
            return self._link_lost_abort()

        # RC override detection ends — we are handing off to AUTO.LAND
        self._in_offboard_mission = False

        if self._auto_land:
            self._phase_note(
                "handing touchdown to PX4 AUTO.LAND — this node stops "
                "commanding the drone from here", warn=True)
            self._stream_on = False
            time.sleep(0.2)
            self._set_mode("AUTO.LAND")
            t0 = time.time()
            while rclpy.ok() and self._armed and time.time() - t0 < 20.0:
                if self._kill_latched:
                    return self._kill_abort()
                self.get_logger().info(
                    f"{self._c('cyan')}[LANDING]{self._c('reset')} AUTO.LAND "
                    f"in progress, waiting for auto-disarm... "
                    f"({20.0 - (time.time() - t0):4.1f} s left)",
                    throttle_duration_sec=2.0)
                time.sleep(0.3)
            if self._armed:
                self._phase_note("land detector slow — forcing disarm", warn=True)
                self._arm(False)
        else:
            self._phase_note("manual disarm near ground...")
            self._arm(False)

        self._mission = self.STATE_DONE
        self._announce_phase(
            "DONE", "MISSION COMPLETE — flew forward, landed & disarmed")  # FORWARD
        self._safe_shutdown()


def main(args=None):
    # TakeoffLandNode names itself "takeoff_land_node". Rename this process's
    # node unless the caller already remapped it (the launch file does).
    argv = list(sys.argv if args is None else args)
    if not any(a.startswith("__node:=") or a.startswith("__name:=") for a in argv):
        argv += ["--ros-args", "-r", "__node:=forward_move_node"]
    rclpy.init(args=argv)
    node = ForwardMoveNode()

    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[FWD] Stopped by user (Ctrl-C).")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
