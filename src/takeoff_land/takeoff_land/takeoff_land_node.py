#!/usr/bin/env python3
"""
takeoff_land_node.py
==================================================================
PURPOSE: Stable TAKE OFF -> HOVER -> LANDING via OFFBOARD mode.
Runs on Jetson Orin Nano, sends position setpoints to PX4 via MAVROS (serial).

ARCHITECTURE (2 nodes):
    px4_sensor_reader  : MAVROS -> /px4/state + /px4/sensors (JSON)
    takeoff_land_node  : reads JSON, sends setpoints to /mavros/setpoint_raw/local

IMPORTANT CONVENTIONS:
    - Setpoints sent to /mavros/setpoint_raw/local, coordinate_frame=FRAME_LOCAL_NED.
      Values are ENU (z up positive); MAVROS converts to NED internally.
    - _wait_ekf_stable() uses std-based check to determine accurate ground_z.
    - Handshake: stream setpoints first -> ARM -> OFFBOARD.

RC OVERRIDE SAFETY:
    If rc_override_enabled=True (default), any unexpected mode change away from
    OFFBOARD during an active mission is treated as RC pilot takeover.
    The mission aborts immediately and setpoints stop — RC pilot has full control.
    PX4 must have COM_RC_OVERRIDE=2 set (via QGroundControl) to actually let
    RC sticks switch the mode.

FSM FLOW:
    IDLE -> (connect) -> (EKF stable -> ground_z) -> (warm-up stream)
         -> ARM -> OFFBOARD -> TAKEOFF (ground_z+target_alt) -> HOVER
         -> LANDING (controlled descent -> AUTO.LAND -> auto-disarm) -> DONE
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String


class TakeoffLandNode(Node):

    STATE_IDLE    = "IDLE"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_HOVER   = "HOVER"
    STATE_LANDING = "LANDING"
    STATE_DONE    = "DONE"

    def __init__(self):
        super().__init__("takeoff_land_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("target_alt",        1.2)    # m above ground
        self.declare_parameter("hover_time",         8.0)    # seconds to hold hover
        self.declare_parameter("cmd_hz",             50)     # setpoint frequency
        self.declare_parameter("use_ekf_stable",     True)
        self.declare_parameter("connect_timeout",    60.0)
        self.declare_parameter("arm_timeout",        30.0)
        self.declare_parameter("descent_speed",      0.3)    # m/s during descent
        self.declare_parameter("land_handoff_alt",   0.25)   # m above ground -> AUTO.LAND
        self.declare_parameter("auto_land_mode",     True)
        self.declare_parameter("rc_override_enabled", True)  # abort if RC takes over
        self.declare_parameter("max_pos_error",      2.0)   # m — max horizontal drift from home
        self.declare_parameter("max_vz",             3.0)   # m/s — max vz during hover/descent
        self.declare_parameter("max_alt_error",      1.5)   # m — max altitude deviation during hover

        self._alt             = float(self.get_parameter("target_alt").value)
        self._hover_time      = float(self.get_parameter("hover_time").value)
        self._cmd_hz          = int(self.get_parameter("cmd_hz").value)
        self._use_ekf_stable  = bool(self.get_parameter("use_ekf_stable").value)
        self._conn_timeout    = float(self.get_parameter("connect_timeout").value)
        self._arm_timeout     = float(self.get_parameter("arm_timeout").value)
        self._descent_speed   = float(self.get_parameter("descent_speed").value)
        self._land_handoff    = float(self.get_parameter("land_handoff_alt").value)
        self._auto_land       = bool(self.get_parameter("auto_land_mode").value)
        self._rc_override_en  = bool(self.get_parameter("rc_override_enabled").value)
        self._max_pos_error   = float(self.get_parameter("max_pos_error").value)
        self._max_vz          = float(self.get_parameter("max_vz").value)
        self._max_alt_error   = float(self.get_parameter("max_alt_error").value)

        # ── Drone state (filled from /px4/state & /px4/sensors) ───────────────
        self._connected = False
        self._armed     = False
        self._mode      = ""
        self._pos       = np.zeros(3)   # x, y, z (ENU local)
        self._vel       = np.zeros(3)
        self._yaw       = 0.0
        self._have_pose = False

        # ── RC override safety ─────────────────────────────────────────────────
        # True while we are actively in TAKEOFF/HOVER/LANDING (OFFBOARD mission).
        # Used to distinguish expected vs. unexpected mode changes.
        self._in_offboard_mission = False
        # Set to True the moment an unexpected mode change is detected.
        self._rc_override         = False

        # ── Streaming setpoint ─────────────────────────────────────────────────
        self._sp_lock   = threading.Lock()
        self._sp_x      = 0.0
        self._sp_y      = 0.0
        self._sp_z      = 0.0
        self._sp_yaw    = 0.0
        self._stream_on = True   # False during AUTO.LAND / after abort

        self._ground_z  = 0.0
        self._home_x    = 0.0
        self._home_y    = 0.0
        self._home_yaw  = 0.0
        self._mission   = self.STATE_IDLE

        # ── Pub / sub / service ────────────────────────────────────────────────
        self._pub_sp = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10)

        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)

        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Timers ─────────────────────────────────────────────────────────────
        self._sp_timer  = self.create_timer(1.0 / self._cmd_hz, self._publish_setpoint)
        self._log_timer = self.create_timer(2.0, self._print_status)

        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"[T/L] takeoff_land_node ready. target_alt={self._alt}m  "
            f"hover={self._hover_time}s  cmd_hz={self._cmd_hz}")
        self.get_logger().info(
            f"[T/L] RC override safety: {'ENABLED' if self._rc_override_en else 'DISABLED'}")
        self.get_logger().info(
            f"[T/L] Sanity limits — pos:{self._max_pos_error:.1f}m  "
            f"vz:{self._max_vz:.1f}m/s  alt_err:{self._max_alt_error:.1f}m")
        self.get_logger().info("=" * 62)

    # ── Callbacks (read JSON from reader) ─────────────────────────────────────
    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            prev_mode       = self._mode
            self._connected = d.get("connected", False)
            self._armed     = d.get("armed",     False)
            self._mode      = d.get("mode",      "")

            # RC override detection: unexpected mode change while we are flying.
            # "AUTO.LAND" is excluded because we set it ourselves during landing.
            # Empty string is excluded to avoid false triggers on startup.
            if (self._rc_override_en and
                    self._in_offboard_mission and
                    prev_mode == "OFFBOARD" and
                    self._mode not in ("OFFBOARD", "AUTO.LAND", "")):
                self._rc_override = True
                self.get_logger().error(
                    f"[RC] MODE CHANGE DETECTED: OFFBOARD -> {self._mode}")
                self.get_logger().error(
                    "[RC] RC OVERRIDE — mission aborting. RC pilot has control.")
        except Exception:
            pass

    def _cb_sensors(self, msg: String):
        try:
            d = json.loads(msg.data)
            if "local_x" in d:
                self._pos = np.array([
                    float(d.get("local_x", 0.0)),
                    float(d.get("local_y", 0.0)),
                    float(d.get("local_z", 0.0)),
                ])
                self._yaw = float(d.get("yaw", 0.0))
                self._have_pose = True
            self._vel = np.array([
                float(d.get("vel_x", 0.0)),
                float(d.get("vel_y", 0.0)),
                float(d.get("vel_z", 0.0)),
            ])
        except Exception:
            pass

    # ── Setpoint streaming (timer) ────────────────────────────────────────────
    def _publish_setpoint(self):
        """Stream position setpoint at cmd_hz. Stops when _stream_on=False."""
        if not self._stream_on:
            return
        msg = PositionTarget()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY  |
            PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        with self._sp_lock:
            msg.position.x = float(self._sp_x)
            msg.position.y = float(self._sp_y)
            msg.position.z = float(self._sp_z)
            msg.yaw        = float(self._sp_yaw)
        self._pub_sp.publish(msg)

    def _set_sp(self, x, y, z, yaw):
        with self._sp_lock:
            self._sp_x, self._sp_y, self._sp_z, self._sp_yaw = x, y, z, yaw

    # ── Service helpers ───────────────────────────────────────────────────────
    def _call_srv(self, client, request, timeout=8.0):
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.05)
        return fut.result()

    def _set_mode(self, mode: str) -> bool:
        if not self._mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("[T/L] /mavros/set_mode not available")
            return False
        req = SetMode.Request(); req.custom_mode = mode
        res = self._call_srv(self._mode_client, req)
        ok = bool(res and res.mode_sent)
        if ok:
            self.get_logger().info(f"[MODE] -> {mode} (sent)")
        return ok

    def _arm(self, value: bool) -> bool:
        if not self._arming_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("[T/L] /mavros/cmd/arming not available")
            return False
        req = CommandBool.Request(); req.value = value
        res = self._call_srv(self._arming_client, req)
        ok = bool(res and res.success)
        if ok:
            self.get_logger().info(f"[ARM] {'ARMED' if value else 'DISARMED'}")
        return ok

    # ── Sanity check (position / velocity / altitude) ─────────────────────────
    def _sanity_abort(self, reason: str):
        """Emergency abort when flight state is unreasonable. Hands off to AUTO.LAND."""
        self._stream_on           = False
        self._in_offboard_mission = False
        self.get_logger().error("=" * 62)
        self.get_logger().error(f"[SANITY] ABORT: {reason}")
        self.get_logger().error("[SANITY] Switching to AUTO.LAND for safety.")
        self.get_logger().error("=" * 62)
        self._set_mode("AUTO.LAND")
        self._safe_shutdown()

    def _sanity_check(self, target_z: float,
                      check_alt: bool = True,
                      check_vz: bool = True) -> bool:
        """
        Returns False and triggers AUTO.LAND abort if any limit is exceeded.
        check_alt=False during takeoff/descent (altitude deviation expected).
        check_vz=False during takeoff (PX4 may legitimately climb at 2-3 m/s).
        """
        x  = float(self._pos[0])
        y  = float(self._pos[1])
        z  = float(self._pos[2])
        vz = abs(float(self._vel[2]))

        dist_xy = math.sqrt((x - self._home_x) ** 2 + (y - self._home_y) ** 2)
        if dist_xy > self._max_pos_error:
            self._sanity_abort(
                f"Horizontal drift {dist_xy:.2f}m from home "
                f"(limit {self._max_pos_error:.1f}m)")
            return False

        if check_vz and vz > self._max_vz:
            self._sanity_abort(
                f"Vertical speed too high: vz={vz:.2f}m/s "
                f"(limit {self._max_vz:.1f}m/s)")
            return False

        if check_alt and abs(z - target_z) > self._max_alt_error:
            self._sanity_abort(
                f"Altitude deviation: z={z:.2f}m target={target_z:.2f}m "
                f"(limit ±{self._max_alt_error:.1f}m)")
            return False

        return True

    # ── RC override abort ─────────────────────────────────────────────────────
    def _rc_override_abort(self):
        """Stop all setpoints immediately and shut down. RC pilot is in control."""
        self._stream_on           = False
        self._in_offboard_mission = False
        self.get_logger().error("=" * 62)
        self.get_logger().error("[T/L] MISSION ABORTED — RC pilot has control.")
        self.get_logger().error("[T/L] Setpoints stopped. Do NOT restart mission")
        self.get_logger().error("[T/L] until drone is safely landed.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    # ── EKF stable wait (std-based) ───────────────────────────────────────────
    def _wait_ekf_stable(self, tol=0.08, stable_dur=5.0, timeout=60.0,
                         pre_wait=10.0) -> float:
        self.get_logger().info(
            f"[T/L] Waiting {pre_wait:.0f}s for GPS/EKF initial convergence...")
        t_pre = time.time()
        while time.time() - t_pre < pre_wait and rclpy.ok():
            time.sleep(0.1)

        self.get_logger().info(
            f"[T/L] Waiting for EKF Z stability (tol={tol}m, dur={stable_dur}s)...")
        t0 = time.time()
        z_hist = []
        stable_start = None
        while time.time() - t0 < timeout and rclpy.ok():
            z_hist.append(float(self._pos[2]))
            if len(z_hist) > 50:
                z_hist.pop(0)
            if len(z_hist) >= 10:
                z_std = float(np.std(z_hist[-10:]))
                if z_std < tol:
                    if stable_start is None:
                        stable_start = time.time()
                    elif time.time() - stable_start >= stable_dur:
                        z_ground = float(np.mean(z_hist[-10:]))
                        self.get_logger().info(
                            f"[T/L] EKF stable. ground_z={z_ground:.3f}m (std={z_std:.4f})")
                        return z_ground
                else:
                    stable_start = None
            time.sleep(0.1)
        z_ground = float(np.mean(z_hist[-10:])) if z_hist else 0.0
        self.get_logger().warn(
            f"[T/L] EKF stable timeout — using ground_z={z_ground:.3f}m")
        return z_ground

    # ── Wait for altitude to stabilize ───────────────────────────────────────
    def _wait_altitude(self, target_z, tol=0.15, vz_tol=0.2,
                       stable_dur=2.0, timeout=30.0) -> bool:
        """Returns True when altitude is stable at target_z.
        Returns False on timeout OR if RC override is detected."""
        t0 = time.time()
        t_stable = None
        while time.time() - t0 < timeout and rclpy.ok():
            if self._rc_override:
                return False
            if not self._sanity_check(target_z, check_alt=False, check_vz=False):
                return False
            z  = float(self._pos[2])
            vz = abs(float(self._vel[2]))
            if abs(z - target_z) < tol and vz < vz_tol:
                if t_stable is None:
                    t_stable = time.time()
                elif time.time() - t_stable >= stable_dur:
                    return True
            else:
                t_stable = None
            time.sleep(0.1)
        return False

    def _print_status(self):
        if not self._connected:
            self.get_logger().warn("[T/L] Waiting for /px4/state (reader & MAVROS)...")
            return
        self.get_logger().info(
            f"[T/L] {self._mission:8s} mode={self._mode:10s} armed={self._armed} "
            f"z={self._pos[2]:.2f}m vz={self._vel[2]:+.2f} sp_z={self._sp_z:.2f}")

    # ── Mission sequence (separate thread) ────────────────────────────────────
    def run_sequence(self):
        # 1. Wait for connection
        self.get_logger().info("[T/L] Waiting for MAVROS connection (via /px4/state)...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > self._conn_timeout:
                self.get_logger().error("[T/L] Connection timeout — aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)
        self.get_logger().info("[T/L] FCU connected.")

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

        # 3. Ground Z
        if self._use_ekf_stable:
            self._ground_z = self._wait_ekf_stable()
        else:
            self._ground_z = float(self._pos[2])
            self.get_logger().info("[T/L] EKF stable wait skipped.")

        # 4. Lock home position
        self._home_x   = float(self._pos[0])
        self._home_y   = float(self._pos[1])
        self._home_yaw = float(self._yaw)
        takeoff_z = self._ground_z + self._alt
        self.get_logger().info(
            f"[T/L] home=({self._home_x:.2f},{self._home_y:.2f}) "
            f"ground_z={self._ground_z:.2f}m -> takeoff_z={takeoff_z:.2f}m")

        self._set_sp(self._home_x, self._home_y, self._ground_z, self._home_yaw)

        # 5. Warm-up setpoint stream
        self.get_logger().info("[T/L] Warming up setpoint stream (~2s)...")
        time.sleep(2.0)

        # 6. ARM
        self.get_logger().info("[T/L] Arming...")
        t0 = time.time(); armed = False
        while rclpy.ok() and time.time() - t0 < self._arm_timeout:
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True
                    break
            time.sleep(1.5)
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
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._mode != "OFFBOARD":
            self.get_logger().error("[T/L] PX4 did not enter OFFBOARD — disarming & aborting.")
            self._arm(False)
            return self._safe_shutdown()

        # RC override detection is active from here until AUTO.LAND handoff
        self._in_offboard_mission = True

        # 8. TAKEOFF
        self._mission = self.STATE_TAKEOFF
        self.get_logger().info(f"[T/L] TAKEOFF -> z={takeoff_z:.2f}m")
        self._set_sp(self._home_x, self._home_y, takeoff_z, self._home_yaw)
        stable = self._wait_altitude(takeoff_z, tol=0.15, timeout=30.0)
        if self._rc_override:
            return self._rc_override_abort()
        if not rclpy.ok():
            return
        if not stable:
            self.get_logger().warn("[T/L] Takeoff not yet stable — continuing carefully.")
        else:
            self.get_logger().info(f"[T/L] Takeoff OK. z={self._pos[2]:.2f}m")

        # 9. HOVER
        self._mission = self.STATE_HOVER
        self.get_logger().info(f"[T/L] HOVER for {self._hover_time:.0f}s...")
        t_h = time.time()
        while rclpy.ok() and time.time() - t_h < self._hover_time:
            if self._rc_override:
                return self._rc_override_abort()
            if not self._sanity_check(takeoff_z, check_alt=True):
                return
            self._set_sp(self._home_x, self._home_y, takeoff_z, self._home_yaw)
            if not self._armed:
                self.get_logger().error("[T/L] Unexpected disarm during hover — aborting.")
                return self._safe_shutdown()
            time.sleep(0.1)

        # 10. LANDING
        self._mission = self.STATE_LANDING
        self.get_logger().info("[T/L] LANDING (controlled descent)...")
        self._controlled_descent(takeoff_z)
        if self._rc_override:
            return self._rc_override_abort()

        # RC override detection ends — we are handing off to AUTO.LAND
        self._in_offboard_mission = False

        if self._auto_land:
            self.get_logger().info("[T/L] Handing touchdown to AUTO.LAND...")
            self._stream_on = False
            time.sleep(0.2)
            self._set_mode("AUTO.LAND")
            t0 = time.time()
            while rclpy.ok() and self._armed and time.time() - t0 < 20.0:
                time.sleep(0.3)
            if self._armed:
                self.get_logger().warn("[T/L] Land detector slow — forcing disarm.")
                self._arm(False)
        else:
            self.get_logger().info("[T/L] Manual disarm near ground...")
            self._arm(False)

        self._mission = self.STATE_DONE
        self.get_logger().info("=" * 62)
        self.get_logger().info("[T/L] MISSION COMPLETE. Drone landed & disarmed.")
        self.get_logger().info("=" * 62)
        self._safe_shutdown()

    def _controlled_descent(self, from_z: float):
        """Lower setpoint z gradually until land_handoff_alt above ground.
        Stops immediately if RC override is detected."""
        dt     = 1.0 / self._cmd_hz
        step   = self._descent_speed * dt
        target_z = self._ground_z + self._land_handoff
        z = from_z
        while rclpy.ok() and z > target_z:
            if self._rc_override:
                return
            if not self._sanity_check(z, check_alt=False):
                return
            z = max(target_z, z - step)
            self._set_sp(self._home_x, self._home_y, z, self._home_yaw)
            if not self._armed:
                self.get_logger().info("[T/L] Auto-disarmed during descent (land detector).")
                return
            time.sleep(dt)
        self.get_logger().info(
            f"[T/L] Descent complete at z={self._pos[2]:.2f}m "
            f"(handoff={self._land_handoff}m above ground).")

    def _safe_shutdown(self):
        self._stream_on = False
        try:
            self._sp_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandNode()

    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[T/L] Stopped by user (Ctrl-C).")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
