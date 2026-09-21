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

NEW (2026-07-08) — mode-switch safety hardening. All additions below are
tagged with "NEW" comments inline so they're easy to find/review:
    1. _publish_setpoint() now also stops the instant _rc_override or
       _link_lost is set, instead of only reacting once the mission-thread
       polling loop notices and flips _stream_on off (closed a small window
       where stale setpoints could still be published after an override was
       already detected).
    2. FCU/MAVROS link loss during an active mission is now detected
       (_link_lost flag, set in _cb_state) and aborts the mission via the
       new _link_lost_abort(), mirroring the existing RC-override path.
       Previously a dropped link during HOVER/LANDING was not handled at all.
    3. run_sequence() now verifies PX4's COM_RC_OVERRIDE parameter
       (_check_rc_override_param()) before a mission is allowed to start.
       Without that param set to 2 or 3 on the flight controller, RC sticks
       cannot physically switch out of OFFBOARD — the detection logic above
       would never fire no matter what this code does.

NEW (2026-09-10) — kill switch & unexpected-disarm safety:
    4. The pilot's KILL switch (RC_MAP_KILL_SW, ch11) is read from
       /mavros/rc/in. From the ARMING phase on, seeing it engaged is latched
       for the rest of the process: setpoints stop at once, the node asks PX4
       for AUTO.LAND (PX4 keeps the vehicle armed for COM_KILL_DISARM = 5 s
       after a kill and restores the motors if the switch is reverted inside
       that window — this makes such a revert resume in LAND, never in this
       OFFBOARD mission), keeps requesting a normal (non-forced) DISARM, which
       PX4 only accepts once landed, and exits. It never arms again and never
       re-enters OFFBOARD.
    5. A disarm this node did not request during the mission is terminal too:
       setpoints stop, no mode change, exit. _wait_altitude() previously had
       no disarm check and kept streaming the takeoff setpoint for up to 30 s.
    6. The node refuses to arm while the kill switch is engaged.

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
from rclpy.qos import (  # NEW: raw RC input is a BEST_EFFORT topic
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)

from mavros_msgs.msg import PositionTarget, RCIn  # NEW: RCIn for the switch check
from mavros_msgs.srv import CommandBool, ParamGet, ParamPull, SetMode  # NEW: param srvs for COM_RC_OVERRIDE check
from rcl_interfaces.msg import ParameterType  # NEW: mavros2 mirrors FCU params as ROS params
from rcl_interfaces.srv import GetParameters  # NEW
from std_msgs.msg import String


# ── Terminal monitoring ──────────────────────────────────────────────────────
# The operator watches this terminal while a pilot holds the RC. Which PHASE
# the drone is in has to be readable at a glance, not reconstructed from a
# stream of INFO lines that all look alike. These names are a MONITORING view
# of the mission — they are announced alongside the real FSM state in
# self._mission and never used to make a decision.
_PHASE_ORDER = ["PREFLIGHT", "READY", "ARMING", "TAKEOFF", "HOVER", "LANDING"]
_PHASE_COLOR = {
    "PREFLIGHT": "cyan",  "READY":   "green",  "ARMING":  "yellow",
    "TAKEOFF":   "yellow", "HOVER":  "blue",   "LANDING": "cyan",
    "DONE":      "green", "ABORT":   "red",
}
_ANSI = {
    "reset": "\033[0m", "bold": "\033[1m", "dim": "\033[2m",
    "red": "\033[91m", "green": "\033[92m", "yellow": "\033[93m",
    "blue": "\033[94m", "cyan": "\033[96m", "white": "\033[97m",
}


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
        self.declare_parameter("verify_rc_override_param", True)  # NEW: verify PX4 COM_RC_OVERRIDE before flying

        # ── NEW: the pilot's mode switch must already be in the OFFBOARD slot
        # before the propellers are allowed to spin. See
        # _check_rc_offboard_position() for why.
        self.declare_parameter("require_rc_offboard",  True)
        self.declare_parameter("rc_mode_channel",      5)      # = RC_MAP_FLTMODE
        self.declare_parameter("rc_offboard_pwm",      1499)   # measured slot centre
        self.declare_parameter("rc_offboard_tol",      150)    # +/- us accepted
        self.declare_parameter("rc_offboard_timeout",  20.0)   # s to wait for it
        # ── NEW (2026-09-10): kill switch. SwF on the AT10II drives ch11 =
        # RC_MAP_KILL_SW; measured 1065 us (off) / 1933 us (on).
        self.declare_parameter("kill_switch_enabled",  True)
        self.declare_parameter("kill_channel",         11)     # = RC_MAP_KILL_SW
        self.declare_parameter("kill_on_pwm",          1500)   # pwm above this = engaged
        self.declare_parameter("max_pos_error",      1.0)   # m — max horizontal drift from home
        self.declare_parameter("max_vz",             3.0)   # m/s — max vz during hover/descent
        self.declare_parameter("max_alt_error",      1.5)   # m — max altitude deviation during hover

        # ── GPS quality pre-arm gate (added after the 2026-08-27 incident) ────
        # WHY THIS EXISTS: on 2026-08-27 the drone armed and took off with
        # fix_type=0 and 0 satellites visible. Nothing in this node objected:
        # _wait_ekf_stable() only looks at the STANDARD DEVIATION of z over a
        # few seconds, so an estimate that is steady-but-wrong (or momentarily
        # quiet while drifting) passes it, and /px4/sensors publishes a
        # local position regardless of whether that position means anything.
        # The commanded setpoint was purely vertical (x/y pinned to home), yet
        # the position estimate jumped, the controller "corrected" toward the
        # phantom error, and the drone drifted sideways into a tree. One run
        # that day tripped the 0.5 m drift limit 4 ms after takeoff — no
        # physical craft moves 1.48 m in 4 ms, that was the estimate jumping.
        #
        # The drift/sanity checks below only fire once the drone is ALREADY
        # airborne, and their abort path (AUTO.LAND) still relies on the same
        # broken position estimate. So the only real fix is to never leave the
        # ground: refuse to ARM until the GPS is actually usable.
        #
        # INDOOR/VIO FLIGHT: set require_gps:=false. The drone then relies on
        # whatever else feeds PX4's local position (VIO, optical flow) and YOU
        # are responsible for confirming that source is healthy.
        self.declare_parameter("require_gps",        True)
        self.declare_parameter("min_fix_type",       3)     # 3 = 3D fix (GPSRAW enum)
        self.declare_parameter("min_satellites",     8)
        self.declare_parameter("max_hdop",           2.0)   # unitless dilution of precision
        self.declare_parameter("gps_wait_timeout",   120.0) # s to wait for a good fix
        self.declare_parameter("gps_stable_dur",     5.0)   # s quality must hold continuously

        # ── EKF ground_z trust criteria (see _wait_ekf_stable) ───────────────
        # max_ground_z is the absolute-value check: we take off FROM THE
        # GROUND, so a local-frame z far from 0 means the estimate is broken,
        # no matter how steady it looks. On 2026-08-27 the old std-only check
        # accepted ground_z = 5.459 m (and another run 8.379 m) for a drone
        # sitting on the ground.
        self.declare_parameter("max_ground_z",       1.0)   # m, |z| allowed on the ground
        self.declare_parameter("ekf_window_s",       5.0)   # s of samples for std/spread
        self.declare_parameter("max_ground_drift",   0.20)  # m peak-to-peak within window

        # ── Terminal monitoring (see _announce_phase) ────────────────────────
        self.declare_parameter("status_period_s",    2.0)   # status line cadence
        self.declare_parameter("color_output",      True)   # ANSI colour

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
        self._verify_rc_param = bool(self.get_parameter("verify_rc_override_param").value)  # NEW
        self._require_rc_offb = bool(self.get_parameter("require_rc_offboard").value)    # NEW
        self._rc_mode_ch      = int(self.get_parameter("rc_mode_channel").value)         # NEW
        self._rc_offb_pwm     = int(self.get_parameter("rc_offboard_pwm").value)         # NEW
        self._rc_offb_tol     = int(self.get_parameter("rc_offboard_tol").value)         # NEW
        self._rc_offb_to      = float(self.get_parameter("rc_offboard_timeout").value)   # NEW
        self._rc_channels     = []                                                       # NEW
        self._kill_en         = bool(self.get_parameter("kill_switch_enabled").value)  # NEW (2026-09-10)
        self._kill_ch         = int(self.get_parameter("kill_channel").value)          # NEW
        self._kill_on_pwm     = int(self.get_parameter("kill_on_pwm").value)           # NEW
        self._kill_pwm        = None    # NEW: last raw value on the kill channel
        self._kill_hits       = 0       # NEW: consecutive "engaged" samples
        self._kill_now        = False   # NEW: live, debounced switch state
        self._kill_watch      = False   # NEW: latching enabled from the ARMING phase on
        self._kill_latched    = False   # NEW: engaged after that -> terminal
        self._max_pos_error   = float(self.get_parameter("max_pos_error").value)
        self._max_vz          = float(self.get_parameter("max_vz").value)
        self._max_alt_error   = float(self.get_parameter("max_alt_error").value)
        self._require_gps     = bool(self.get_parameter("require_gps").value)
        self._min_fix_type    = int(self.get_parameter("min_fix_type").value)
        self._min_sats        = int(self.get_parameter("min_satellites").value)
        self._max_hdop        = float(self.get_parameter("max_hdop").value)
        self._gps_wait_to     = float(self.get_parameter("gps_wait_timeout").value)
        self._gps_stable_dur  = float(self.get_parameter("gps_stable_dur").value)
        self._max_ground_z    = float(self.get_parameter("max_ground_z").value)
        self._ekf_window_s    = float(self.get_parameter("ekf_window_s").value)
        self._max_gnd_drift   = float(self.get_parameter("max_ground_drift").value)
        self._status_period   = float(self.get_parameter("status_period_s").value)
        self._color           = bool(self.get_parameter("color_output").value)

        # ── Drone state (filled from /px4/state & /px4/sensors) ───────────────
        self._connected = False
        self._armed     = False
        self._mode      = ""
        self._pos       = np.zeros(3)   # x, y, z (ENU local)
        self._vel       = np.zeros(3)
        self._yaw       = 0.0
        self._have_pose = False
        # GPS quality from /px4/sensors ("gps_quality" block). None = the
        # reader has not sent one yet, which the pre-arm gate treats as
        # "not proven good" rather than as "fine".
        self._gps_q     = None

        # ── RC override safety ─────────────────────────────────────────────────
        # True while we are actively in TAKEOFF/HOVER/LANDING (OFFBOARD mission).
        # Used to distinguish expected vs. unexpected mode changes.
        self._in_offboard_mission = False
        # Set to True the moment an unexpected mode change is detected.
        self._rc_override         = False
        # NEW (2026-09-10): a disarm this node did not request during the mission.
        self._disarm_abort        = False
        # NEW: Set to True the moment the FCU/MAVROS link drops during a mission.
        self._link_lost           = False

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

        # ── Phase / monitoring state ─────────────────────────────────────────
        self._phase_idx   = 0
        self._phase_name  = "PREFLIGHT"
        self._t_run_start = time.time()   # T+ shown on every banner/status line
        self._t_phase     = time.time()

        # ── Pub / sub / service ────────────────────────────────────────────────
        self._pub_sp = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10)

        self.create_subscription(String, "/px4/state",   self._cb_state,   10)
        self.create_subscription(String, "/px4/sensors", self._cb_sensors, 10)

        # NEW: raw RC channels for _check_rc_offboard_position(). MAVROS
        # publishes this BEST_EFFORT, so the QoS has to match or nothing
        # ever arrives.
        self.create_subscription(
            RCIn, "/mavros/rc/in", self._cb_rc_in,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5,
                       durability=DurabilityPolicy.VOLATILE))

        self._arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client   = self.create_client(SetMode,     "/mavros/set_mode")

        # ── Timers ─────────────────────────────────────────────────────────────
        self._sp_timer  = self.create_timer(1.0 / self._cmd_hz, self._publish_setpoint)
        self._log_timer = self.create_timer(
            self._status_period, self._print_status)

        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"[T/L] takeoff_land_node ready. target_alt={self._alt}m  "
            f"hover={self._hover_time}s  cmd_hz={self._cmd_hz}")
        self.get_logger().info(
            f"[T/L] RC override safety: {'ENABLED' if self._rc_override_en else 'DISABLED'}")
        self.get_logger().info(
            f"[T/L] Sanity limits — pos:{self._max_pos_error:.1f}m  "
            f"vz:{self._max_vz:.1f}m/s  alt_err:{self._max_alt_error:.1f}m")
        if self._require_gps:
            self.get_logger().info(
                f"[T/L] GPS pre-arm gate: ENABLED — fix>=3D, "
                f"sats>={self._min_sats}, HDOP<={self._max_hdop:.1f}")
        else:
            self.get_logger().warn(
                "[T/L] GPS pre-arm gate: DISABLED (require_gps:=false) — "
                "indoor/VIO only!")
        self.get_logger().info("=" * 62)

    # ── Callbacks (read JSON from reader) ─────────────────────────────────────
    def _cb_state(self, msg: String):
        try:
            d = json.loads(msg.data)
            prev_mode       = self._mode
            prev_connected  = self._connected  # NEW
            prev_armed      = self._armed      # NEW (2026-09-10)
            self._connected = d.get("connected", False)
            self._armed     = d.get("armed",     False)
            self._mode      = d.get("mode",      "")

            # NEW (2026-09-10): a disarm during the mission that this node did
            # not request (kill-switch timeout, PX4 failsafe, the pilot's arm
            # switch) ends the mission. _in_offboard_mission is cleared before
            # every hand-off where a disarm is expected (AUTO.LAND).
            if (self._in_offboard_mission and prev_armed and not self._armed
                    and not self._disarm_abort):
                self._disarm_abort = True
                self._stream_on = False
                self.get_logger().error(
                    "[ARM] UNEXPECTED DISARM during active mission — "
                    "setpoints stopped.")

            # NEW: FCU/MAVROS link-loss detection during an active mission.
            # If telemetry drops while we're supposed to be commanding the
            # vehicle, we no longer know its real mode — stop touching it and
            # let PX4's own failsafe (data-link-loss / RC-loss) take over.
            if self._in_offboard_mission and prev_connected and not self._connected:
                self._link_lost = True
                self.get_logger().error(
                    "[LINK] FCU DISCONNECTED during active mission!")

            # RC override detection: unexpected mode change while we are flying.
            # AUTO.LAND is NOT excluded any more (2026-09-09). Every place this
            # node sets AUTO.LAND itself clears _in_offboard_mission FIRST, so
            # the guard above already covers that case. Excluding the mode as
            # well meant a pilot flicking the switch to Land went undetected:
            # setpoints kept streaming, and flicking back to OFFBOARD silently
            # resumed the mission mid-descent.
            # Empty string is excluded to avoid false triggers on startup.
            if (self._rc_override_en and
                    self._in_offboard_mission and
                    prev_mode == "OFFBOARD" and
                    self._mode not in ("OFFBOARD", "")):
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
            q = d.get("gps_quality")
            if isinstance(q, dict):
                self._gps_q = q
        except Exception:
            pass

    # ── Setpoint streaming (timer) ────────────────────────────────────────────
    def _publish_setpoint(self):
        """Stream position setpoint at cmd_hz. Stops when _stream_on=False."""
        # NEW: also stop the instant RC override or link-loss is detected,
        # instead of waiting for the mission thread's polling loop to notice
        # and flip _stream_on off (previously up to ~0.1s of stale setpoints
        # could still go out after the override was already known about).
        if (not self._stream_on or self._rc_override or self._link_lost
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
        self._announce_phase("ABORT", f"SANITY: {reason}", warn=True)
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
        self._announce_phase(
            "ABORT", "RC OVERRIDE — pilot has control", warn=True)
        self.get_logger().error("=" * 62)
        self.get_logger().error("[T/L] MISSION ABORTED — RC pilot has control.")
        self.get_logger().error("[T/L] Setpoints stopped. Do NOT restart mission")
        self.get_logger().error("[T/L] until drone is safely landed.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    # ── NEW: FCU link-loss abort ──────────────────────────────────────────────
    def _link_lost_abort(self):
        """NEW: Stop all setpoints immediately when the FCU/telemetry link
        drops during an active mission. Mirrors _rc_override_abort(), but for
        a lost connection instead of a detected mode change — we don't know
        the vehicle's real mode once telemetry is gone, so the safest move is
        to stop commanding it and let PX4's own failsafe take over."""
        self._stream_on           = False
        self._in_offboard_mission = False
        self._announce_phase("ABORT", "FCU LINK LOST", warn=True)
        self.get_logger().error("=" * 62)
        self.get_logger().error("[T/L] MISSION ABORTED — FCU link lost.")
        self.get_logger().error("[T/L] Setpoints stopped. PX4 failsafe should take over.")
        self.get_logger().error("[T/L] Do NOT restart mission until link is restored")
        self.get_logger().error("[T/L] and drone is confirmed safely landed.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    # ── NEW: read a PX4 parameter via MAVROS ──────────────────────────────────
    def _get_px4_param_int(self, name: str, timeout=5.0):
        """NEW: read an integer PX4 parameter through MAVROS.
        mavros2 (ROS2 Humble) has NO /mavros/param/get service — FCU params
        are mirrored as ROS parameters on the /mavros/param node instead, so
        we use the standard /mavros/param/get_parameters interface. The old
        mavros1-style /mavros/param/get is kept as a fallback. Returns the
        integer value, or None if it could not be read."""
        client = self.create_client(GetParameters, "/mavros/param/get_parameters")
        if client.wait_for_service(timeout_sec=timeout):
            req = GetParameters.Request()
            req.names = [name]
            res = self._call_srv(client, req, timeout=timeout)
            if res and res.values:
                v = res.values[0]
                if v.type == ParameterType.PARAMETER_INTEGER:
                    return int(v.integer_value)
                if v.type == ParameterType.PARAMETER_DOUBLE:
                    return int(v.double_value)
        # fallback: legacy mavros1-style service (not present on Humble mavros2)
        client = self.create_client(ParamGet, "/mavros/param/get")
        if client.wait_for_service(timeout_sec=2.0):
            req = ParamGet.Request()
            req.param_id = name
            res = self._call_srv(client, req, timeout=timeout)
            if res and res.success:
                return int(res.value.integer or res.value.real)
        return None

    # ── NEW: verify PX4 safety parameter before flying ────────────────────────
    def _check_rc_override_param(self) -> bool:
        """NEW: confirm PX4's COM_RC_OVERRIDE parameter actually allows the RC
        pilot to switch out of OFFBOARD mid-flight. Without this set correctly
        on the flight controller, the RC-override detection in _cb_state() is
        unenforceable — the sticks physically cannot regain control no matter
        what this code does. Returns True if verified OK, or if the check
        could not be performed and was skipped; False only when the value is
        confirmed wrong."""
        if not self._rc_override_en or not self._verify_rc_param:
            return True

        # Make sure the FCU param table is synced to MAVROS first — right
        # after boot the mirror may still be empty (pull over serial is slow).
        pull = self.create_client(ParamPull, "/mavros/param/pull")
        if pull.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("[SAFETY] Syncing PX4 params (param/pull)...")
            self._call_srv(pull, ParamPull.Request(force_pull=False), timeout=60.0)

        # Retry for a while: the param may simply not be mirrored yet.
        value = None
        t0 = time.time()
        while value is None and rclpy.ok() and time.time() - t0 < 30.0:
            value = self._get_px4_param_int("COM_RC_OVERRIDE")
            if value is None:
                time.sleep(3.0)

        if value is None:
            self.get_logger().warn(
                "[SAFETY] Could not read COM_RC_OVERRIDE — cannot verify. "
                "Proceeding, but VERIFY MANUALLY in QGC.")
            return True
        if value not in (2, 3):
            self.get_logger().error("=" * 62)
            self.get_logger().error(
                f"[SAFETY] COM_RC_OVERRIDE={value} on the flight controller — "
                "RC sticks CANNOT take over OFFBOARD mode with this setting!")
            self.get_logger().error(
                "[SAFETY] Set it to 2 in QGroundControl (Parameters) and "
                "reboot PX4 before flying this mission.")
            self.get_logger().error("=" * 62)
            return False
        self.get_logger().info(f"[SAFETY] COM_RC_OVERRIDE={value} — verified OK.")

        # Informational: what PX4 will do if our setpoint stream stops
        # (offboard-loss failsafe). Meaning of the value depends on PX4
        # version — confirm in QGC that it is Hold / Land / Return.
        obl = self._get_px4_param_int("COM_OBL_RC_ACT", timeout=3.0)
        if obl is not None:
            self.get_logger().info(
                f"[SAFETY] COM_OBL_RC_ACT={obl} (offboard-loss failsafe action — "
                "confirm in QGC this maps to Hold/Land/Return for your PX4 version).")
        return True

    # ── NEW: require the RC mode switch to sit in the OFFBOARD slot ──────────

    def _cb_rc_in(self, msg):
        """Raw RC channels: the OFFBOARD-slot check and the kill switch."""
        try:
            self._rc_channels = list(msg.channels)
        except Exception:
            return
        self._update_kill_switch(self._rc_channels)

    # ── NEW (2026-09-10): kill switch ─────────────────────────────────────────
    def _update_kill_switch(self, ch):
        """Debounced kill-switch state from the raw RC channels.

        Two consecutive samples above kill_on_pwm count as engaged, so one
        corrupted frame cannot end a mission. A false positive is safe anyway:
        it only stops this node's setpoints and asks PX4 for LAND — it never
        cuts the motors. Once _kill_watch is set (ARMING phase), the first
        engaged state is latched for the rest of this process."""
        if not self._kill_en:
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
                "setpoints stopped. This node will only request LAND + DISARM "
                "from now on.")

    def _kill_abort(self):
        """Terminal reaction to the kill switch (pilot's decision, 2026-09-10).

        PX4 v1.17 keeps the vehicle ARMED for COM_KILL_DISARM (5 s) after a
        kill and restores the motors if the switch is reverted inside that
        window. So: 1) setpoints are already stopped, 2) ask for AUTO.LAND so
        that a revert resumes into LAND and never into this OFFBOARD mission
        (PX4's offboard-loss failsafe COM_OBL_RC_ACT would get there too, after
        COM_OF_LOSS_T), 3) keep asking for a normal DISARM, which PX4 only
        accepts once landed, 4) exit without ever arming or entering OFFBOARD
        again."""
        self._stream_on           = False
        self._in_offboard_mission = False
        self._announce_phase(
            "ABORT", "KILL SWITCH — motors cut by the pilot", warn=True)
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[KILL] MISSION ABORTED — kill switch engaged. Requesting AUTO.LAND "
            "(a revert inside PX4's kill window lands) and DISARM.")
        self.get_logger().error(
            "[KILL] This node will NOT arm or enter OFFBOARD again.")
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
        self._stream_on           = False
        self._in_offboard_mission = False
        self._announce_phase("ABORT", "UNEXPECTED DISARM", warn=True)
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[ARM] MISSION ABORTED — the drone was disarmed without this node "
            "asking. Setpoints stopped; no mode change; no re-arm.")
        self.get_logger().error("=" * 62)
        self._safe_shutdown()

    def _check_kill_switch_released(self) -> bool:
        """Refuse to arm while the kill switch is engaged or unreadable."""
        if not self._kill_en:
            self.get_logger().warn(
                "[SAFETY] kill_switch_enabled:=false — the kill switch is NOT "
                "watched by this node.")
            return True
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and time.time() - t0 < self._rc_offb_to:
            now = time.time()
            if self._kill_pwm is not None and not self._kill_now:
                self.get_logger().info(
                    f"[SAFETY] Kill switch released "
                    f"(ch{self._kill_ch}={self._kill_pwm}). OK.")
                return True
            if now - last >= 3.0:
                last = now
                if self._kill_pwm is None:
                    self.get_logger().warn(
                        f"[SAFETY] No RC data for kill channel ch{self._kill_ch} "
                        f"yet ({self._rc_offb_to - (now - t0):.0f}s left)")
                else:
                    self.get_logger().warn(
                        f"[SAFETY] KILL SWITCH IS ENGAGED "
                        f"(ch{self._kill_ch}={self._kill_pwm}) — release it "
                        f"({self._rc_offb_to - (now - t0):.0f}s left)")
            time.sleep(0.2)
        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[SAFETY] KILL SWITCH ENGAGED OR UNREADABLE — REFUSING TO ARM.")
        self.get_logger().error("=" * 62)
        return False

    def _check_rc_offboard_position(self) -> bool:
        """Refuse to arm unless the pilot's mode switch is ALREADY in the
        OFFBOARD slot.

        WHY: this node asks PX4 for OFFBOARD over MAVLink, and PX4 grants it
        no matter where the RC mode switch happens to be. Flown that way the
        switch and the actual flight mode disagree, and the pilot's first
        "take over" flick can land on the position the switch is already in
        — a no-op at the exact moment it matters. Starting in OFFBOARD makes
        every other switch position a live escape route.
        """
        if not self._require_rc_offb:
            self.get_logger().warn(
                "[SAFETY] require_rc_offboard:=false — the RC mode switch "
                "position is NOT checked. Switch and flight mode may "
                "disagree.")
            return True

        lo = self._rc_offb_pwm - self._rc_offb_tol
        hi = self._rc_offb_pwm + self._rc_offb_tol
        idx = self._rc_mode_ch - 1
        self.get_logger().info(
            f"[SAFETY] Waiting for the RC mode switch to be in OFFBOARD "
            f"(ch{self._rc_mode_ch} within {lo}-{hi})...")
        t0 = time.time()
        last = 0.0
        while rclpy.ok() and time.time() - t0 < self._rc_offb_to:
            ch = self._rc_channels
            now = time.time()
            if ch and 0 <= idx < len(ch):
                pwm = int(ch[idx])
                if lo <= pwm <= hi:
                    self.get_logger().info(
                        f"[SAFETY] RC mode switch is in OFFBOARD "
                        f"(ch{self._rc_mode_ch}={pwm}). OK.")
                    return True
                if now - last >= 3.0:
                    last = now
                    self.get_logger().warn(
                        f"[SAFETY] Move the RC mode switch to OFFBOARD: "
                        f"ch{self._rc_mode_ch}={pwm}, need {lo}-{hi} "
                        f"({self._rc_offb_to - (now - t0):.0f}s left)")
            elif now - last >= 3.0:
                last = now
                self.get_logger().warn(
                    f"[SAFETY] No RC data on /mavros/rc/in yet "
                    f"({self._rc_offb_to - (now - t0):.0f}s left)")
            time.sleep(0.2)

        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[SAFETY] RC MODE SWITCH IS NOT IN THE OFFBOARD SLOT — "
            "REFUSING TO ARM.")
        self.get_logger().error(
            "[SAFETY] Put the switch in OFFBOARD before starting, so that "
            "flicking it anywhere else is a real takeover.")
        self.get_logger().error("=" * 62)
        return False

    # ── GPS quality pre-arm gate ──────────────────────────────────────────────
    def _gps_quality_problem(self):
        """Return None when GPS quality is good enough to fly, else a short
        human-readable reason string. Missing/unknown values count as BAD:
        an unknown fix is exactly the situation that put the drone in a tree
        on 2026-08-27, so it must never be treated as acceptable."""
        q = self._gps_q
        if not q:
            return "no GPS quality data from /px4/sensors yet"

        fix = q.get("fix_type")
        if fix is None:
            return "fix_type unknown"
        if int(fix) < self._min_fix_type:
            names = {0: "NO GPS", 1: "NO FIX", 2: "2D fix", 3: "3D fix",
                     4: "DGPS", 5: "RTK-float", 6: "RTK-fixed"}
            return (f"fix_type={names.get(int(fix), fix)} "
                    f"(need >= {self._min_fix_type} = 3D fix)")

        sats = q.get("satellites")
        if sats is None:
            return "satellite count unknown"
        if int(sats) < self._min_sats:
            return f"only {int(sats)} satellites (need >= {self._min_sats})"

        hdop = q.get("hdop")
        if hdop is None:
            return "HDOP unknown"
        if float(hdop) > self._max_hdop:
            return f"HDOP {float(hdop):.2f} too high (need <= {self._max_hdop:.2f})"

        return None

    def _wait_gps_quality(self) -> bool:
        """Block until GPS quality is good CONTINUOUSLY for gps_stable_dur.

        Requiring it to hold (rather than sampling once) is deliberate: a
        receiver that is still acquiring flickers in and out of a valid fix,
        and a single lucky sample is how a marginal GPS talks its way past a
        gate like this one. Returns False on timeout -> caller must abort."""
        if not self._require_gps:
            self.get_logger().warn("=" * 62)
            self.get_logger().warn(
                "[GPS] require_gps:=false — GPS pre-arm gate DISABLED.")
            self.get_logger().warn(
                "[GPS] Only do this indoors with a healthy VIO/optical-flow "
                "source feeding PX4's local position. YOU are responsible for "
                "verifying it.")
            self.get_logger().warn("=" * 62)
            return True

        self.get_logger().info(
            f"[GPS] Pre-arm gate: need fix>={self._min_fix_type} (3D), "
            f"sats>={self._min_sats}, HDOP<={self._max_hdop:.2f}, "
            f"held for {self._gps_stable_dur:.0f}s "
            f"(timeout {self._gps_wait_to:.0f}s)...")

        t0 = time.time()
        t_good = None
        last_report = 0.0
        while rclpy.ok() and time.time() - t0 < self._gps_wait_to:
            problem = self._gps_quality_problem()
            now = time.time()
            if problem is None:
                if t_good is None:
                    t_good = now
                    q = self._gps_q or {}
                    self.get_logger().info(
                        f"[GPS] Quality OK (sats={q.get('satellites')}, "
                        f"HDOP={q.get('hdop')}) — holding "
                        f"{self._gps_stable_dur:.0f}s to confirm...")
                elif now - t_good >= self._gps_stable_dur:
                    q = self._gps_q or {}
                    self.get_logger().info(
                        f"[GPS] Pre-arm gate PASSED — fix_type={q.get('fix_type')}, "
                        f"sats={q.get('satellites')}, HDOP={q.get('hdop')}, "
                        f"h_acc={q.get('h_acc_m')}m")
                    return True
            else:
                if t_good is not None:
                    self.get_logger().warn(
                        f"[GPS] Quality lost again: {problem} — restarting hold.")
                t_good = None
                if now - last_report >= 5.0:
                    last_report = now
                    self.get_logger().warn(
                        f"[GPS] Waiting for usable GPS: {problem} "
                        f"({self._gps_wait_to - (now - t0):.0f}s left)")
            time.sleep(0.2)

        self.get_logger().error("=" * 62)
        self.get_logger().error(
            f"[GPS] PRE-ARM GATE FAILED: {self._gps_quality_problem()}")
        self.get_logger().error(
            "[GPS] REFUSING TO ARM. Flying on a bad position estimate is how "
            "the drone drifted into a tree on 2026-08-27.")
        self.get_logger().error(
            "[GPS] Move to open sky away from trees/buildings and retry, or "
            "set require_gps:=false ONLY if a healthy VIO/optical-flow source "
            "is providing PX4's local position.")
        self.get_logger().error("=" * 62)
        return False

    # ── EKF stable wait (value + spread + drift) ──────────────────────────────
    def _wait_ekf_stable(self, tol=0.08, stable_dur=5.0, timeout=60.0,
                         pre_wait=10.0):
        """Determine ground_z, or return None if the estimate cannot be trusted.

        HOW THE OLD VERSION WAS FOOLED (2026-08-27): it took the standard
        deviation of the last TEN samples — a ONE SECOND window — and nothing
        else. On the ground that day the EKF z was swinging roughly +-5 m over
        ~50 s (logged: 4.99 -> 0.87 -> 10.07 -> 5.45 while the drone sat
        still). At the turning points of that slow swing the one-second spread
        is tiny, so std came out 0.0166 and the check declared "stable" with
        ground_z = 5.459 m for a drone sitting on the ground. Takeoff then
        proceeded on an estimate that was metres wrong, the controller chased
        the phantom error sideways, and the drone ended up in a tree.

        Three independent criteria must now hold CONTINUOUSLY for stable_dur,
        because each one catches a failure the others miss:

          a) std over the window < tol
                 the original check — catches fast jitter.
          b) peak-to-peak over the window <= max_ground_drift
                 catches the SLOW drift that a short-window std cannot see.
                 A one-second std of 0.0166 says nothing about a metre of
                 travel over the preceding half minute.
          c) |mean z| <= max_ground_z
                 the absolute-value check. We take off FROM THE GROUND, so a
                 local-frame z of 5.46 m (or -7.18 m, also logged that day) is
                 not "stable", it is proof the estimate is broken. Steadiness
                 alone can never establish correctness — a wrong number that
                 sits still is still wrong.

        On timeout this now returns None instead of handing back whatever the
        mean happened to be. The old behaviour ("could not confirm the EKF, so
        fly on it anyway") is exactly backwards for a safety check.
        """
        self.get_logger().info(
            f"[T/L] Waiting {pre_wait:.0f}s for GPS/EKF initial convergence...")
        t_pre = time.time()
        while time.time() - t_pre < pre_wait and rclpy.ok():
            time.sleep(0.1)

        n_win = max(10, int(self._ekf_window_s / 0.1))   # samples in the window
        self.get_logger().info(
            f"[T/L] Waiting for EKF Z: std<{tol}m, spread<={self._max_gnd_drift}m "
            f"over {self._ekf_window_s:.1f}s, |z|<={self._max_ground_z}m, "
            f"held {stable_dur:.0f}s (timeout {timeout:.0f}s)...")

        t0 = time.time()
        z_hist = []
        stable_start = None
        last_report = 0.0
        while time.time() - t0 < timeout and rclpy.ok():
            z_hist.append(float(self._pos[2]))
            if len(z_hist) > n_win:
                z_hist.pop(0)

            problem = None
            if len(z_hist) < n_win:
                problem = f"collecting samples ({len(z_hist)}/{n_win})"
            else:
                win     = z_hist[-n_win:]
                z_std   = float(np.std(win))
                z_pp    = float(max(win) - min(win))
                z_mean  = float(np.mean(win))
                if z_std > tol:
                    problem = f"jitter std={z_std:.3f}m (need <{tol}m)"
                elif z_pp > self._max_gnd_drift:
                    problem = (f"drifting {z_pp:.2f}m over "
                               f"{self._ekf_window_s:.0f}s "
                               f"(need <={self._max_gnd_drift}m)")
                elif abs(z_mean) > self._max_ground_z:
                    problem = (f"z={z_mean:.2f}m but we are ON THE GROUND "
                               f"(need |z|<={self._max_ground_z}m)")

            now = time.time()
            if problem is None:
                if stable_start is None:
                    stable_start = now
                elif now - stable_start >= stable_dur:
                    win = z_hist[-n_win:]
                    z_ground = float(np.mean(win))
                    self.get_logger().info(
                        f"[T/L] EKF stable. ground_z={z_ground:.3f}m "
                        f"(std={float(np.std(win)):.4f}, "
                        f"spread={float(max(win) - min(win)):.3f}m)")
                    return z_ground
            else:
                stable_start = None
                if now - last_report >= 5.0:
                    last_report = now
                    self.get_logger().warn(
                        f"[T/L] EKF not usable yet: {problem} "
                        f"({timeout - (now - t0):.0f}s left)")
            time.sleep(0.1)

        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[T/L] EKF NEVER BECAME TRUSTWORTHY within the timeout.")
        if z_hist:
            win = z_hist[-min(len(z_hist), n_win):]
            self.get_logger().error(
                f"[T/L] Last window: z={float(np.mean(win)):.2f}m, "
                f"spread={float(max(win) - min(win)):.2f}m, "
                f"std={float(np.std(win)):.3f}m")
        self.get_logger().error(
            "[T/L] REFUSING TO FLY on an unverified position estimate — this "
            "is what put the drone in a tree on 2026-08-27.")
        self.get_logger().error("=" * 62)
        return None

    # ── Wait for altitude to stabilize ───────────────────────────────────────
    def _wait_altitude(self, target_z, tol=0.15, vz_tol=0.2,
                       stable_dur=2.0, timeout=30.0) -> bool:
        """Returns True when altitude is stable at target_z.
        Returns False on timeout OR if RC override / link loss is detected."""
        t0 = time.time()
        t_stable = None
        while time.time() - t0 < timeout and rclpy.ok():
            if self._rc_override:
                return False
            if self._link_lost:  # NEW
                return False
            if self._kill_latched or self._disarm_abort:  # NEW (2026-09-10)
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

    # ── Terminal monitoring: phase banners + rich status line ────────────────

    def _c(self, code: str) -> str:
        """ANSI escape, or '' when colour is disabled."""
        return _ANSI.get(code, "") if self._color else ""

    def _elapsed(self) -> str:
        s = int(time.time() - self._t_run_start)
        return f"T+{s // 60:02d}:{s % 60:02d}"

    def _emit(self, text: str, warn: bool = False):
        """Log at INFO or WARN from two SEPARATE call sites.

        rclpy caches logger state per call site and raises 'Logger severity
        cannot be changed between calls' if one site is used at two
        severities. Selecting the method into a variable and calling it from a
        single line does exactly that. Keep these two calls on their own
        lines."""
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

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

    def _print_status(self):
        """Flight-monitoring status line.

        Shows PHYSICAL altitude (z - ground_z) as well as raw local-frame z:
        raw z alone is meaningless to an operator standing next to the drone,
        and misreading it is exactly what went unnoticed on 2026-08-27."""
        col  = self._c(_PHASE_COLOR.get(self._phase_name, "white"))
        rst  = self._c("reset")
        bold = self._c("bold")
        if not self._connected:
            self.get_logger().warn(
                f"[{self._elapsed()}] {self._phase_name} — waiting for "
                "/px4/state (px4_sensor_reader & MAVROS)...")
            return

        alt = float(self._pos[2]) - self._ground_z
        parts = [f"alt={alt:5.2f}m", f"z={float(self._pos[2]):6.2f}m",
                 f"vz={float(self._vel[2]):+5.2f}"]

        if self._home_x or self._home_y:
            d_home = math.sqrt((float(self._pos[0]) - self._home_x) ** 2
                               + (float(self._pos[1]) - self._home_y) ** 2)
            mark = ""
            if self._max_pos_error > 0.0:
                frac = d_home / self._max_pos_error
                if frac > 0.8:
                    mark = self._c("red") + "!" + rst
                elif frac > 0.6:
                    mark = self._c("yellow") + "." + rst
            parts.append(f"drift={d_home:5.2f}m{mark}")

        parts.append(f"sp_z={float(self._sp_z):5.2f}")
        arm_c = self._c("red") if self._armed else self._c("green")
        parts.append(f"{arm_c}{'ARMED' if self._armed else 'disarmed'}{rst}")
        parts.append(f"mode={self._mode or '?'}")
        if self._kill_now:  # NEW (2026-09-10)
            parts.append(f"{self._c('red')}KILL{rst}")

        if self._gps_q:
            sats = self._gps_q.get("satellites")
            hdop = self._gps_q.get("hdop")
            parts.append(f"gps={'?' if sats is None else sats}sat/"
                         f"{'?' if hdop is None else f'{float(hdop):.1f}'}")

        self.get_logger().info(
            f"{col}{bold}[{self._elapsed()}] {self._phase_name:<9s}{rst} | "
            + " | ".join(parts))

    # ── Mission sequence (separate thread) ────────────────────────────────────
    def run_sequence(self):
        self._t_run_start = time.time()
        self._announce_phase(
            "PREFLIGHT",
            "FCU -> COM_RC_OVERRIDE -> local pose -> GPS gate -> ground_z")
        # 1. Wait for connection
        self.get_logger().info("[T/L] Waiting for MAVROS connection (via /px4/state)...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > self._conn_timeout:
                self.get_logger().error("[T/L] Connection timeout — aborting.")
                return self._safe_shutdown()
            time.sleep(0.2)
        self.get_logger().info("[T/L] FCU connected.")

        # NEW: verify COM_RC_OVERRIDE before continuing — see _check_rc_override_param().
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

        # 2b. GPS quality gate — BEFORE ground_z/home are latched, because both
        # are derived from the position estimate we are about to trust for the
        # whole flight. See _wait_gps_quality() for the incident this prevents.
        if not self._wait_gps_quality():
            self.get_logger().error("[T/L] Aborting — GPS not safe to fly on.")
            return self._safe_shutdown()

        # 3. Ground Z
        if self._use_ekf_stable:
            ground_z = self._wait_ekf_stable()
            # None = the estimate never became trustworthy. Refusing here is
            # the whole point: everything downstream (takeoff_z, home, every
            # sanity limit) is measured against this number.
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
        takeoff_z = self._ground_z + self._alt
        self.get_logger().info(
            f"[T/L] home=({self._home_x:.2f},{self._home_y:.2f}) "
            f"ground_z={self._ground_z:.2f}m -> takeoff_z={takeoff_z:.2f}m")

        self._set_sp(self._home_x, self._home_y, self._ground_z, self._home_yaw)

        # 4b. Everything that can be verified BEFORE the propellers spin has
        # now been verified. Announce it as its own phase so the operator has
        # one unambiguous "the drone is about to fly" moment to react to,
        # instead of having to notice it from the log flow.
        self._announce_phase(
            "READY",
            f"DRONE READY TO LAUNCH — GPS ok, ground_z={self._ground_z:.2f} m, "
            f"home=({self._home_x:.2f},{self._home_y:.2f}), "
            f"takeoff_z={takeoff_z:.2f} m (physical ~{self._alt:.1f} m)")
        self._phase_note(
            f"hover {self._hover_time:.0f} s, then descend at "
            f"{self._descent_speed:.2f} m/s to {self._land_handoff:.2f} m "
            f"and hand off to AUTO.LAND")
        self._phase_note(
            "PILOT: hold the RC with the mode switch ready. Next step ARMS "
            "the drone.", warn=True)

        # 5b. NEW: the pilot's mode switch must already be in the OFFBOARD
        # slot, so that every other switch position stays a live escape route.
        if not self._check_rc_offboard_position():
            return self._safe_shutdown()

        # 5c. NEW (2026-09-10): never arm with the kill switch engaged; from
        # here on any engagement of it is terminal (see _kill_abort).
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
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()

        # 6. ARM
        self._phase_note("sending ARM...")
        t0 = time.time(); armed = False
        while rclpy.ok() and time.time() - t0 < self._arm_timeout:
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True
                    break
            time.sleep(1.5)
        if self._kill_latched:  # NEW (2026-09-10)
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
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._mode != "OFFBOARD":
            self.get_logger().error("[T/L] PX4 did not enter OFFBOARD — disarming & aborting.")
            self._arm(False)
            return self._safe_shutdown()

        # RC override detection is active from here until AUTO.LAND handoff
        self._in_offboard_mission = True
        if not self._armed:  # NEW (2026-09-10): disarmed before takeoff began
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
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._disarm_abort:  # NEW (2026-09-10)
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()
        if self._link_lost:  # NEW
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
            f"z={takeoff_z:.2f} m before landing")
        t_h = time.time()
        while rclpy.ok() and time.time() - t_h < self._hover_time:
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            if self._disarm_abort:  # NEW (2026-09-10)
                return self._disarm_stop()
            if self._rc_override:
                return self._rc_override_abort()
            if self._link_lost:  # NEW
                return self._link_lost_abort()
            if not self._sanity_check(takeoff_z, check_alt=True):
                return
            self._set_sp(self._home_x, self._home_y, takeoff_z, self._home_yaw)
            self.get_logger().info(
                f"{self._c('blue')}[HOVER]{self._c('reset')} "
                f"landing in {self._hover_time - (time.time() - t_h):4.1f} s...",
                throttle_duration_sec=2.0)
            if not self._armed:
                self._announce_phase(
                    "ABORT", "unexpected DISARM during hover", warn=True)
                return self._safe_shutdown()
            time.sleep(0.1)

        # 10. LANDING
        self._mission = self.STATE_LANDING
        self._announce_phase(
            "LANDING",
            f"controlled descent at {self._descent_speed:.2f} m/s to "
            f"{self._land_handoff:.2f} m above ground, then AUTO.LAND")
        self._controlled_descent(takeoff_z)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._disarm_abort:  # NEW (2026-09-10)
            return self._disarm_stop()
        if self._rc_override:
            return self._rc_override_abort()
        if self._link_lost:  # NEW
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
                if self._kill_latched:  # NEW (2026-09-10): stream is already off
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
            "DONE", "MISSION COMPLETE — drone landed & disarmed")
        self._safe_shutdown()

    def _controlled_descent(self, from_z: float):
        """Lower setpoint z gradually until land_handoff_alt above ground.
        Stops immediately if RC override is detected."""
        dt     = 1.0 / self._cmd_hz
        step   = self._descent_speed * dt
        target_z = self._ground_z + self._land_handoff
        z = from_z
        while rclpy.ok() and z > target_z:
            if self._kill_latched or self._disarm_abort:  # NEW (2026-09-10)
                return
            if self._rc_override:
                return
            if self._link_lost:  # NEW
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
