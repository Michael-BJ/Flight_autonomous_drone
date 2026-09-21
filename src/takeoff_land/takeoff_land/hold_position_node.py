#!/usr/bin/env python3
"""
hold_position_node.py
=====================
Emergency OFFBOARD re-entry after RC override or manual mode.

SCENARIO:
    1. Mission was running in OFFBOARD mode.
    2. RC pilot took over (mode changed to POSCTL / MANUAL / etc).
    3. Drone is stable in the air under RC control.
    4. You want to re-enter OFFBOARD, hold position, then land safely.

WHAT THIS NODE DOES:
    1. Reads current position from /px4/sensors.
    2. Streams setpoints at that position (hold point).
    3. Switches mode to OFFBOARD — drone holds in place.
    4. Holds for `hold_time` seconds.
    5. Executes controlled descent -> AUTO.LAND -> disarm.

USAGE:
    # Default: hold 5s then land
    ros2 run takeoff_land hold_position_node

    # Hold 10s before landing
    ros2 run takeoff_land hold_position_node --ros-args -p hold_time:=10.0

    # Hold only (no auto-land) — disarm manually via RC
    ros2 run takeoff_land hold_position_node --ros-args \
        -p hold_time:=30.0 -p auto_land:=false

REQUIREMENTS:
    - MAVROS and px4_sensor_reader must already be running.
    - Drone must be armed and in the air.
    - Setpoints will stream from startup; switch to OFFBOARD after ~2s.

RC OVERRIDE SAFETY (2026-07-27):
    Once OFFBOARD is confirmed, any unexpected mode change away from it
    (RC pilot switching to POSCTL/MANUAL/etc) is latched permanently in
    _rc_override — it is never cleared. The setpoint publisher checks
    this flag directly, so streaming stops the instant the override is
    seen, and every step of run_sequence() (hold loop, descent, AUTO.LAND
    handoff) re-checks it before doing anything further. This closes a
    prior bug where switching mode back to OFFBOARD via RC could cause
    this node to instantly resume commanding the last hold/descent
    setpoint — an unwanted autonomous movement.

KILL SWITCH / UNEXPECTED DISARM (2026-09-10):
    The kill switch (RC_MAP_KILL_SW, ch11, read from /mavros/rc/in) is watched
    for the whole life of this node — it only ever runs with the drone in the
    air. Engaged (two consecutive samples) = latched: setpoints stop at once,
    the node asks PX4 for AUTO.LAND (a revert inside PX4's 5 s COM_KILL_DISARM
    window then resumes in LAND, never in this node's OFFBOARD hold), keeps
    requesting a normal DISARM (PX4 only accepts it once landed) and exits.
    A disarm while holding OFFBOARD that this node did not request is
    terminal too: setpoints stop, no mode change, exit.
"""
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (  # NEW (2026-09-10): raw RC input is BEST_EFFORT
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

from mavros_msgs.msg import PositionTarget, RCIn  # NEW (2026-09-10): kill switch
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String


class HoldPositionNode(Node):

    def __init__(self):
        super().__init__("hold_position_node")

        self.declare_parameter("hold_time",       5.0)   # seconds to hold before landing
        self.declare_parameter("descent_speed",   0.2)   # m/s
        self.declare_parameter("land_handoff_alt", 0.25) # m above ground -> AUTO.LAND
        self.declare_parameter("auto_land",       True)   # hand off to AUTO.LAND
        self.declare_parameter("cmd_hz",          50)
        # NEW (2026-09-10): kill switch. AT10II SwF -> ch11 = RC_MAP_KILL_SW,
        # measured 1065 us (off) / 1933 us (on).
        self.declare_parameter("kill_switch_enabled", True)
        self.declare_parameter("kill_channel",        11)
        self.declare_parameter("kill_on_pwm",         1500)

        self._hold_time     = float(self.get_parameter("hold_time").value)
        self._descent_speed = float(self.get_parameter("descent_speed").value)
        self._land_handoff  = float(self.get_parameter("land_handoff_alt").value)
        self._auto_land     = bool(self.get_parameter("auto_land").value)
        self._cmd_hz        = int(self.get_parameter("cmd_hz").value)
        self._kill_en       = bool(self.get_parameter("kill_switch_enabled").value)  # NEW
        self._kill_ch       = int(self.get_parameter("kill_channel").value)          # NEW
        self._kill_on_pwm   = int(self.get_parameter("kill_on_pwm").value)           # NEW
        self._kill_pwm      = None    # NEW: last raw value on the kill channel
        self._kill_hits     = 0       # NEW: consecutive "engaged" samples
        self._kill_now      = False   # NEW: live, debounced switch state
        self._kill_watch    = True    # NEW: this node only runs airborne -> watch from start
        self._kill_latched  = False   # NEW: engaged -> terminal
        self._disarm_abort  = False   # NEW: disarm we did not request while holding

        # State from sensor reader
        self._connected = False
        self._armed     = False
        self._mode      = ""
        self._pos       = np.zeros(3)
        self._vel       = np.zeros(3)
        self._yaw       = 0.0
        self._have_pose = False

        # RC override safety — mirrors takeoff_land_node.py.
        # True once we've confirmed OFFBOARD is active (arms the detector).
        self._in_offboard = False
        # Latched True the instant an unexpected mode change away from
        # OFFBOARD is seen. Never reset back to False — once the RC pilot
        # takes the mode away from us, this node must never command
        # movement or mode changes again for the rest of its life.
        self._rc_override = False

        # Hold target (locked on startup)
        self._hold_x   = 0.0
        self._hold_y   = 0.0
        self._hold_z   = 0.0
        self._hold_yaw = 0.0

        self._sp_lock   = threading.Lock()
        self._sp_x      = 0.0
        self._sp_y      = 0.0
        self._sp_z      = 0.0
        self._sp_yaw    = 0.0
        self._stream_on = True

        self._pub_sp = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10)

        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)
        # NEW (2026-09-10): raw RC for the kill switch (BEST_EFFORT, as MAVROS).
        self.create_subscription(
            RCIn, "/mavros/rc/in", self._cb_rc_in,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5,
                       durability=DurabilityPolicy.VOLATILE))

        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")

        self._sp_timer = self.create_timer(1.0 / self._cmd_hz, self._publish_sp)

        self.get_logger().info("=" * 62)
        self.get_logger().info("[HOLD] hold_position_node ready.")
        self.get_logger().info(
            f"[HOLD] Will hold current position for {self._hold_time:.0f}s then land.")
        self.get_logger().info("=" * 62)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            prev_mode        = self._mode
            prev_armed       = self._armed   # NEW (2026-09-10)
            self._connected  = d.get("connected", False)
            self._armed      = d.get("armed",     False)
            self._mode       = d.get("mode",      "")

            # NEW (2026-09-10): a disarm while holding OFFBOARD that this node
            # did not request is terminal (_in_offboard is cleared before the
            # AUTO.LAND hand-off, where a disarm is expected).
            if (self._in_offboard and prev_armed and not self._armed
                    and not self._disarm_abort):
                self._disarm_abort = True
                self._stream_on = False
                self.get_logger().error(
                    "[HOLD] UNEXPECTED DISARM while holding — setpoints stopped.")

            # RC override detection: unexpected mode change while we hold
            # OFFBOARD. AUTO.LAND is NOT excluded any more (2026-09-09): the
            # final handoff clears _in_offboard FIRST, so the guard above
            # already covers it, and a pilot flicking the switch to Land now
            # aborts the hold like any other takeover.
            if (self._in_offboard and
                    prev_mode == "OFFBOARD" and
                    self._mode not in ("OFFBOARD", "")):
                if not self._rc_override:
                    self.get_logger().error(
                        f"[HOLD] MODE CHANGE DETECTED: OFFBOARD -> {self._mode}")
                    self.get_logger().error(
                        "[HOLD] RC OVERRIDE — holding aborted. RC pilot has control.")
                self._rc_override = True
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

    # ── NEW (2026-09-10): kill switch ─────────────────────────────────────────

    def _cb_rc_in(self, msg):
        """Debounced kill-switch state from the raw RC channels. Two
        consecutive samples above kill_on_pwm = engaged; latched for good."""
        if not self._kill_en:
            return
        try:
            ch = list(msg.channels)
        except Exception:
            return
        idx = self._kill_ch - 1
        if not (0 <= idx < len(ch)):
            return
        pwm = int(ch[idx])
        self._kill_pwm = pwm
        self._kill_hits = self._kill_hits + 1 if pwm > self._kill_on_pwm else 0
        self._kill_now = self._kill_hits >= 2
        if self._kill_now and self._kill_watch and not self._kill_latched:
            self._kill_latched = True
            self._stream_on = False
            self.get_logger().error(
                f"[KILL] KILL SWITCH ENGAGED (ch{self._kill_ch}={pwm}) — "
                "setpoints stopped. Only LAND + DISARM will be requested.")

    def _kill_abort(self):
        """Terminal reaction to the kill switch: setpoints already stopped;
        ask for AUTO.LAND (so a revert inside PX4's kill window lands); keep
        requesting a normal DISARM (PX4 only accepts it once landed); exit
        without ever entering OFFBOARD again."""
        self._stream_on   = False
        self._in_offboard = False
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[KILL] HOLD ABORTED — kill switch engaged. Requesting AUTO.LAND "
            "and DISARM. This node will NOT command the drone otherwise.")
        self.get_logger().error("=" * 62)
        if self._armed:
            for _ in range(3):
                if self._set_mode("AUTO.LAND"):
                    break
                time.sleep(0.3)
        t0 = time.time()
        while rclpy.ok() and self._armed and time.time() - t0 < 15.0:
            self._arm(False)   # normal disarm: PX4 rejects it while airborne
            time.sleep(1.0)
        if self._armed:
            self.get_logger().error(
                "[KILL] Still ARMED after 15 s — PX4 did not accept DISARM "
                "(not landed yet?). Pilot: use the RC.")
        else:
            self.get_logger().error("[KILL] Drone is DISARMED.")
        self._safe_shutdown()

    def _disarm_stop(self):
        """Terminal reaction to a disarm this node did not request."""
        self._stream_on   = False
        self._in_offboard = False
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[HOLD] ABORTED — the drone was disarmed without this node asking. "
            "Setpoints stopped; no mode change.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    # ── setpoint stream ───────────────────────────────────────────────────────

    def _publish_sp(self):
        # Stop the instant an override is latched, don't wait for the
        # mission thread's polling loop to notice and flip _stream_on off.
        if (not self._stream_on or self._rc_override
                or self._kill_latched or self._disarm_abort):  # NEW (2026-09-10)
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

    # ── service helpers ───────────────────────────────────────────────────────

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
            return False
        req = SetMode.Request(); req.custom_mode = mode
        res = self._call_srv(self._mode_client, req)
        ok = bool(res and res.mode_sent)
        if ok:
            self.get_logger().info(f"[HOLD] Mode -> {mode}")
        return ok

    def _arm(self, value: bool) -> bool:
        if not self._arming_client.wait_for_service(timeout_sec=3.0):
            return False
        req = CommandBool.Request(); req.value = value
        res = self._call_srv(self._arming_client, req)
        ok = bool(res and res.success)
        if ok:
            self.get_logger().info(f"[HOLD] {'ARMED' if value else 'DISARMED'}")
        return ok

    def _safe_shutdown(self):
        self._stream_on = False
        try:
            self._sp_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    def _rc_override_abort(self):
        """Stop all setpoints immediately and shut down. RC pilot is in
        control — this node must not touch mode or setpoints again."""
        self._stream_on = False
        self.get_logger().error("=" * 62)
        self.get_logger().error("[HOLD] ABORTED — RC pilot has control.")
        self.get_logger().error("[HOLD] Setpoints stopped. Do NOT restart hold_position_node")
        self.get_logger().error("[HOLD] until the drone is safely landed.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    # ── main sequence ─────────────────────────────────────────────────────────

    def run_sequence(self):
        # 1. Wait for MAVROS connection
        self.get_logger().info("[HOLD] Waiting for MAVROS connection...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > 30.0:
                self.get_logger().error("[HOLD] Connection timeout — aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)
        self.get_logger().info(f"[HOLD] Connected. Current mode: {self._mode}")

        # 2. Wait for position data
        t0 = time.time()
        while rclpy.ok() and not self._have_pose:
            if time.time() - t0 > 15.0:
                self.get_logger().error("[HOLD] No position data — aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)

        # 3. Lock hold position at current location
        self._hold_x   = float(self._pos[0])
        self._hold_y   = float(self._pos[1])
        self._hold_z   = float(self._pos[2])
        self._hold_yaw = float(self._yaw)
        self.get_logger().info(
            f"[HOLD] Hold point locked: "
            f"x={self._hold_x:.2f}m  y={self._hold_y:.2f}m  z={self._hold_z:.2f}m")

        # 4. Stream setpoints at hold point
        self._set_sp(self._hold_x, self._hold_y, self._hold_z, self._hold_yaw)
        self.get_logger().info("[HOLD] Streaming setpoints at hold point (~2s warmup)...")
        time.sleep(2.0)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()

        # 5. Switch to OFFBOARD
        self.get_logger().info("[HOLD] Switching to OFFBOARD mode...")
        if not self._set_mode("OFFBOARD"):
            self.get_logger().error("[HOLD] Failed to enter OFFBOARD — aborting.")
            return self._safe_shutdown()

        t0 = time.time()
        while rclpy.ok() and self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()

        if self._mode != "OFFBOARD":
            self.get_logger().error(
                "[HOLD] PX4 did not enter OFFBOARD.\n"
                "       Tip: setpoints must flow >2 Hz before switching mode.\n"
                "       Make sure drone is armed and in a switchable state.")
            return self._safe_shutdown()

        self.get_logger().info(
            f"[HOLD] OFFBOARD active. Holding position for {self._hold_time:.0f}s...")

        # RC override detection is active from here until AUTO.LAND handoff.
        self._in_offboard = True

        # 6. Hold loop
        t_hold = time.time()
        while rclpy.ok() and time.time() - t_hold < self._hold_time:
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            if self._disarm_abort:  # NEW (2026-09-10)
                return self._disarm_stop()
            if self._rc_override:
                return self._rc_override_abort()

            self._set_sp(self._hold_x, self._hold_y, self._hold_z, self._hold_yaw)

            elapsed = time.time() - t_hold
            remaining = self._hold_time - elapsed
            self.get_logger().info(
                f"[HOLD] mode={self._mode}  z={self._pos[2]:.2f}m  "
                f"landing in {remaining:.0f}s...",
                throttle_duration_sec=2.0)

            if not self._armed:
                self.get_logger().warn("[HOLD] Drone disarmed unexpectedly.")
                return self._safe_shutdown()

            time.sleep(0.1)

        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._disarm_abort:  # NEW (2026-09-10)
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()

        # 7. Controlled descent
        self.get_logger().info("[HOLD] Starting descent...")
        target_z = self._land_handoff
        self._descent(self._hold_z, target_z)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._disarm_abort:  # NEW (2026-09-10)
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()

        # RC override detection ends — we are handing off to AUTO.LAND
        self._in_offboard = False

        # 8. AUTO.LAND handoff
        if self._auto_land:
            self.get_logger().info("[HOLD] Handing off to AUTO.LAND...")
            self._stream_on = False
            time.sleep(0.2)
            self._set_mode("AUTO.LAND")
            t0 = time.time()
            while rclpy.ok() and self._armed and time.time() - t0 < 25.0:
                if self._kill_latched:  # NEW (2026-09-10)
                    return self._kill_abort()
                time.sleep(0.3)
            if self._armed:
                self.get_logger().warn("[HOLD] Land detector slow — forcing disarm.")
                self._arm(False)
        else:
            self._arm(False)

        self.get_logger().info("=" * 62)
        self.get_logger().info("[HOLD] Landed and disarmed. Done.")
        self.get_logger().info("=" * 62)
        self._safe_shutdown()

    def _descent(self, from_z: float, target_z: float):
        dt   = 1.0 / self._cmd_hz
        step = self._descent_speed * dt
        z    = from_z
        while rclpy.ok() and z > target_z:
            if self._kill_latched or self._disarm_abort:  # NEW (2026-09-10)
                return
            if self._rc_override:
                return
            if not self._armed:
                self.get_logger().info("[HOLD] Disarmed during descent.")
                return
            z = max(target_z, z - step)
            self._set_sp(self._hold_x, self._hold_y, z, self._hold_yaw)
            time.sleep(dt)
        self.get_logger().info(f"[HOLD] Descent complete at z={self._pos[2]:.2f}m.")


def main(args=None):
    rclpy.init(args=args)
    node = HoldPositionNode()

    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[HOLD] Stopped by user.")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
