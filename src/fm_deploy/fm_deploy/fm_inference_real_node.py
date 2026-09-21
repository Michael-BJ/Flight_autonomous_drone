#!/usr/bin/env python3
# fm_inference_real_node.py — v1.0 — FM-Planner inference on REAL hardware
"""
fm_inference_real_node.py — FM-Planner deployment on hardware
==========================================================================
This node = `FMInferenceNode` (exactly the version already proven in
simulation) + the HARDWARE SAFETY LAYER taken from `takeoff_land_node.py`
(a program that has already flown for real).

DESIGN PRINCIPLE — and why:

  The candidate generator, ranking, MINCO, ESDF, guards, and escape are
  NOT touched at all; everything is inherited as-is from fm_inference_base /
  fm_inference_node. If the planning logic were changed for hardware, real
  flight results could no longer be compared against simulation results,
  weakening the paper's "same model" claim. The ONLY things added here are
  what genuinely doesn't exist in simulation:

    1. RC OVERRIDE  — the pilot can take control at any time. The moment
       the mode switches away from OFFBOARD, setpoints STOP instantly
       (not after the mission loop gets around to checking).
    2. LINK LOSS    — MAVROS telemetry drops mid-flight -> stop sending
       setpoints and let PX4's failsafe take over.
    3. GEOFENCE     — distance limit from the home point + altitude
       deviation limit. In Gazebo, flying out of the arena only corrupts
       data; in the field it hits a person.
    4. BATTERY      — abort + land before voltage drops too low.
    5. HOME FRAME   — in simulation the drone always spawns at (0,0)
       facing +X, so goal (goal_x, 0) is correct as-is. In the field the
       EKF origin is at whatever position & heading it happens to have at
       boot. The goal here is defined RELATIVE to home:
       goal = home + R(yaw_home) . [goal_dist, 0]. The arena/geofence is
       likewise computed from home, not from absolute coordinates.
    6. DRY RUN      — runs the ENTIRE pipeline (camera -> octomap -> ESDF
       -> FM -> MINCO -> RViz markers) WITHOUT arming and WITHOUT
       setpoints. This is the correct way to run the first test: battery
       plugged in, propellers OFF, drone held/on a bench, then watch the
       "[FM] GATE two-sided ..." log and the candidate markers. If
       anything looks off here, never move past it.
    7. TAKEOFF/LANDING hold XY HOME (not the instantaneous position like
       in sim). Holding the instantaneous position means every bit of EKF
       drift becomes part of the command — the drone slowly "chases" its
       own drift while climbing.
    8. KILL SWITCH (NEW 2026-09-10) — the pilot's kill switch (RC_MAP_KILL_SW,
       ch11, read from /mavros/rc/in) is latched once the ARM step begins:
       setpoints stop at once, the node asks PX4 for AUTO.LAND (PX4 keeps the
       vehicle armed for COM_KILL_DISARM = 5 s after a kill and restores the
       motors on a revert inside that window — this makes the revert resume in
       LAND, never in this OFFBOARD mission), keeps requesting a normal DISARM
       (PX4 only accepts it once landed) and exits. It never arms again. Arming
       is refused while the switch is engaged.
    9. UNEXPECTED DISARM (NEW 2026-09-10) — a disarm during the mission that
       this node did not request stops setpoints immediately and ends the
       mission WITHOUT any mode change (it used to go through the landing
       path, which kept streaming setpoints).

FSM ORDER:
    IDLE -> connect -> check COM_RC_OVERRIDE -> wait for pose -> wait for depth+ESDF
         -> EKF stable (ground_z) -> lock home + compute goal & geofence
         -> warm up model + stream setpoints -> ARM -> OFFBOARD
         -> TAKEOFF -> settle + reset octomap -> FLYING (FM replan)
         -> LANDING (controlled descent -> AUTO.LAND) -> DONE

USAGE:
    ros2 launch fm_deploy fm_real.launch.py \
        model_path:=$HOME/drone_ws/src/fm_deploy/model/fm/fm_planner_XXXX.onnx \
        goal_dist:=6.0 target_alt:=1.2 v_max:=0.5 dry_run:=true
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from mavros_msgs.msg import PositionTarget, RCIn  # NEW: RCIn for the switch check
from rclpy.qos import (  # NEW: raw RC input is a BEST_EFFORT topic
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy)
from mavros_msgs.srv import ParamGet, ParamPull
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter as RclParameter            # NEW (2026-09-13): octomap band
from rcl_interfaces.msg import ParameterValue as RclParameterValue  # NEW (2026-09-13): octomap band
from rcl_interfaces.srv import GetParameters, SetParameters         # NEW (2026-09-13): SetParameters
from std_srvs.srv import Empty as EmptySrv

import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from fm_inference_node import FMInferenceNode


# ── Terminal monitoring (hardware only) ──────────────────────────────────────
# The mission FSM in fm_inference_base has no HOVER state: the settle window
# after takeoff runs while still in STATE_TAKEOFF, and it is silent for
# hover_settle_s + ~1.5 s. From the ground that is indistinguishable from a
# hang. These phase names are a MONITORING view of the mission — they are
# announced alongside the real FSM state, never used to make a decision.
_PHASE_ORDER = ["PREFLIGHT", "ARMING", "TAKEOFF", "HOVER", "FLYING", "LANDING"]
_PHASE_COLOR = {
    "PREFLIGHT": "cyan",   "ARMING":  "yellow", "TAKEOFF": "yellow",
    "HOVER":     "blue",   "FLYING":  "green",  "LANDING": "cyan",
    "DONE":      "green",  "ABORT":   "red",    "DRY RUN": "blue",
}
_ANSI = {
    "reset": "\033[0m", "bold": "\033[1m", "dim": "\033[2m",
    "red": "\033[91m", "green": "\033[92m", "yellow": "\033[93m",
    "blue": "\033[94m", "cyan": "\033[96m", "white": "\033[97m",
}


class _FlyingYawLockPublisher:
    """NEW (2026-09-15): wraps the PositionTarget publisher. While FLYING
    (trajectory tracking, not escape) the yaw is replaced by the home yaw,
    so the nose/camera stays on the home->goal direction instead of
    following atan2(trajectory velocity), which swung up to 180 deg between
    replans on 9/15 00:53 and left the camera looking 83 deg off the path.
    Every other field and every other phase passes through unchanged."""

    def __init__(self, real_pub, node):
        self._real = real_pub
        self._node = node
        self._ref_last = None        # NEW (2026-09-16, RTHYAW)
        # NEW (2026-09-15, YAWSMOOTH): state of the "smooth" mode, all yaw
        # offsets in rad relative to the home yaw. None = not active.
        self._t_last = None
        self._off_filt = 0.0   # low-passed wanted offset
        self._off_held = 0.0   # offset after the deadband
        self._off_cmd = 0.0    # rate-limited offset actually sent
        self._tracking = False # deadband exceeded, following until settled

    # NEW (2026-09-16, RTHYAW): the yaw the lock holds. _yaw_lock_ref is set
    # per leg by _fly_to_target (home yaw outbound, bearing to home on the
    # RETURN leg), so the nose follows the direction of travel both ways.
    def _ref(self, n):
        ref = float(getattr(n, "_yaw_lock_ref", n._home_yaw))
        if self._ref_last is None or abs(self._wrap(ref - self._ref_last)) > 1e-6:
            self._ref_last = ref
            self._t_last = None          # restart the smooth slew on this leg
        return ref

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _smooth_yaw(self, n, msg):
        """NEW (2026-09-15, YAWSMOOTH): nose follows the trajectory direction,
        low-passed, deadbanded, rate-limited and kept within
        +-yaw_max_offset of the home->goal direction."""
        now = time.monotonic()
        max_off = n._yaw_max_off
        if self._t_last is None or now - self._t_last > 0.5:
            # (re)entering FLYING: start from where the nose points now
            cur = float(np.clip(self._wrap(n._drone_state.yaw - self._ref(n)),
                                -max_off, max_off))
            self._off_filt = self._off_held = self._off_cmd = cur
            self._tracking = False
            dt = 0.0
        else:
            dt = min(now - self._t_last, 0.1)
        self._t_last = now
        vx, vy = float(msg.velocity.x), float(msg.velocity.y)
        raw = self._wrap(math.atan2(vy, vx) - self._ref(n))
        # moving mostly backwards (> 120 deg off the goal direction): hold,
        # otherwise the clamp would flip the nose between +max and -max
        if (dt > 0.0 and math.hypot(vx, vy) > n._yaw_min_speed
                and abs(raw) <= math.radians(120.0)):
            want = float(np.clip(raw, -max_off, max_off))
            k = 1.0 - math.exp(-dt / n._yaw_tau) if n._yaw_tau > 0 else 1.0
            self._off_filt += k * (want - self._off_filt)
            # deadband as hysteresis: small wobble never starts a turn, but
            # once started the nose follows until the filter has settled
            # (a plain deadband would stop up to deadband short of the path)
            if abs(self._off_filt - self._off_held) > n._yaw_deadband:
                self._tracking = True
            if self._tracking:
                self._off_held = self._off_filt
                if abs(want - self._off_filt) < math.radians(2.0):
                    self._off_held = want
                    self._tracking = False
        # slow / hovering: keep holding the last heading
        step = n._yaw_rate_max * dt
        self._off_cmd += float(np.clip(self._off_held - self._off_cmd,
                                       -step, step))
        return self._wrap(self._ref(n) + self._off_cmd)

    def publish(self, msg):
        n = self._node
        try:
            active = (n._home_locked
                      and n._mission_state == n.STATE_FLYING
                      and not n._escape_active
                      and isinstance(msg, PositionTarget)
                      and not (msg.type_mask & PositionTarget.IGNORE_YAW))
            if active and n._flying_yaw_mode == "home":
                msg.yaw = float(self._ref(n))
            elif active and n._flying_yaw_mode == "smooth":
                msg.yaw = float(self._smooth_yaw(n, msg))
            elif not active:
                self._t_last = None
        except Exception:
            # never let bookkeeping stop the setpoint stream
            pass
        return self._real.publish(msg)

    def __getattr__(self, name):
        return getattr(self._real, name)


class FMInferenceRealNode(FMInferenceNode):

    # ── Construction ─────────────────────────────────────────────────────────

    def __init__(self):
        # The parent sets up the model, ESDF, MINCO, subscribers, and the
        # setpoint timer.
        super().__init__()

        # ── Hardware-specific parameters ─────────────────────────────────────
        # Goal in the HOME FRAME (see point 5 in the docstring). The
        # parent's goal_x is ignored when use_home_frame=true.
        self.declare_parameter("use_home_frame", True)
        self.declare_parameter("goal_dist",      6.0)    # m, forward from home
        self.declare_parameter("goal_lat",       0.0)    # m, shift left(+)/right(-)
        # Geofence (home frame). The ESDF arena box is computed from this.
        self.declare_parameter("fence_fwd",      12.0)   # m ahead of home
        self.declare_parameter("fence_back",      3.0)   # m behind home
        self.declare_parameter("fence_lat",       4.0)   # m left/right of home
        self.declare_parameter("max_home_dist",  15.0)   # m, hard abort radius
        self.declare_parameter("max_alt_error",   1.0)   # m, z deviation -> abort
        # Battery
        self.declare_parameter("min_battery_v",   0.0)   # V, 0 = disabled
        self.declare_parameter("min_battery_pct", 0.0)   # %, 0 = disabled
        # ── GPS quality pre-arm gate (added after the 2026-08-27 incident) ────
        # WHY THIS EXISTS: on 2026-08-27 a drone on this airframe armed and
        # took off with fix_type=0 and 0 satellites visible. Nothing objected:
        # _wait_ekf_stable() only looks at the STANDARD DEVIATION of z over a
        # few seconds, so an estimate that is steady-but-wrong passes it, and
        # /px4/sensors publishes a local position regardless of whether that
        # position means anything. The commanded setpoint was purely vertical,
        # yet the position estimate jumped, the controller "corrected" toward
        # the phantom error, and the drone drifted sideways into a tree.
        #
        # The in-flight watchdog below only fires once the drone is ALREADY
        # airborne, and its abort path still relies on the same broken position
        # estimate. So the only real fix is to never leave the ground.
        #
        # INDOOR/VIO FLIGHT: set require_gps:=false. The drone then relies on
        # whatever else feeds PX4's local position (VIO, optical flow) and YOU
        # are responsible for confirming that source is healthy.
        self.declare_parameter("require_gps",      True)
        self.declare_parameter("min_fix_type",     3)     # 3 = 3D fix (GPSRAW enum)
        self.declare_parameter("min_satellites",   8)
        self.declare_parameter("max_hdop",         2.0)   # unitless dilution of precision
        self.declare_parameter("gps_wait_timeout", 120.0) # s to wait for a good fix
        self.declare_parameter("gps_stable_dur",   5.0)   # s quality must hold continuously

        # ── EKF ground_z trust criteria (see _wait_ekf_stable below) ─────────
        # max_ground_z is the absolute-value check: we take off FROM THE
        # GROUND, so a local-frame z far from 0 means the estimate is broken,
        # no matter how steady it looks. On 2026-08-27 a std-only check
        # accepted ground_z = 5.459 m (and 8.379 m on another run) for a drone
        # sitting on the ground.
        self.declare_parameter("max_ground_z",     1.0)   # m, |z| allowed on the ground
        self.declare_parameter("ekf_window_s",     5.0)   # s of samples for std/spread
        self.declare_parameter("max_ground_drift", 0.20)  # m peak-to-peak within window
        # RC / link
        self.declare_parameter("rc_override_enabled",       True)
        self.declare_parameter("verify_rc_override_param",  True)

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
        # Bench test without flying
        self.declare_parameter("dry_run",        False)
        # Mission
        self.declare_parameter("mission_timeout_s", 180.0)
        self.declare_parameter("hover_settle_s",      5.0)
        self.declare_parameter("descent_speed",       0.3)
        self.declare_parameter("land_handoff_alt",    0.25)
        self.declare_parameter("auto_land_mode",     True)
        # Writing PX4 parameters. In simulation the parent writes
        # EKF2_HGT_REF=1 (height reference = GPS). In the field that CAN BE
        # WRONG: if you fly indoor with VIO/optical-flow, forcing GPS as the
        # height reference corrupts the estimate. Default: DO NOT touch PX4
        # parameters.
        self.declare_parameter("write_px4_params", False)
        self.declare_parameter("ekf2_hgt_ref",        -1)   # <0 = not written
        # ── Terminal monitoring (hardware only — see _announce_phase) ────────
        # In the field the operator watches this terminal while a pilot holds
        # the RC. Which PHASE the drone is in has to be readable at a glance,
        # not reconstructed from a stream of INFO lines that all look alike.
        self.declare_parameter("status_period_s",    2.0)   # status line cadence
        self.declare_parameter("color_output",      True)   # ANSI colour
        # Warn when a replan overruns planning_time_ahead. That parameter IS
        # the compute-time budget: the new trajectory starts at where the
        # drone was predicted to be planning_time_ahead into the future, but
        # it is played back from the moment it is installed. Overrunning it
        # means the trajectory is born already behind the drone.
        self.declare_parameter("warn_replan_overrun", True)
        # Delay before starting to measure EKF Z std. In simulation the EKF
        # converges instantly (doesn't need this). In the field, the EKF
        # only starts stabilizing a few seconds after PX4 gets a GPS/VIO
        # fix — without this pre-wait, the std window can "accidentally"
        # look stable while the EKF is still moving toward its final value
        # (ground_z gets recorded wrong, and that offset carries through the
        # whole mission). Same as takeoff_land_node.py.
        self.declare_parameter("ekf_pre_wait_s",      10.0)

        self._use_home_frame = bool(self.get_parameter("use_home_frame").value)
        self._goal_dist   = float(self.get_parameter("goal_dist").value)
        self._goal_lat    = float(self.get_parameter("goal_lat").value)
        self._fence_fwd   = float(self.get_parameter("fence_fwd").value)
        self._fence_back  = float(self.get_parameter("fence_back").value)
        self._fence_lat   = float(self.get_parameter("fence_lat").value)
        self._max_home_d  = float(self.get_parameter("max_home_dist").value)
        self._max_alt_err = float(self.get_parameter("max_alt_error").value)
        self._min_batt_v  = float(self.get_parameter("min_battery_v").value)
        self._min_batt_p  = float(self.get_parameter("min_battery_pct").value)
        self._require_gps    = bool(self.get_parameter("require_gps").value)
        self._min_fix_type   = int(self.get_parameter("min_fix_type").value)
        self._min_sats       = int(self.get_parameter("min_satellites").value)
        self._max_hdop       = float(self.get_parameter("max_hdop").value)
        self._gps_wait_to    = float(self.get_parameter("gps_wait_timeout").value)
        self._gps_stable_dur = float(self.get_parameter("gps_stable_dur").value)
        self._max_ground_z   = float(self.get_parameter("max_ground_z").value)
        self._ekf_window_s   = float(self.get_parameter("ekf_window_s").value)
        self._max_gnd_drift  = float(self.get_parameter("max_ground_drift").value)
        self._rc_ovr_en   = bool(self.get_parameter("rc_override_enabled").value)
        self._verify_rc   = bool(self.get_parameter("verify_rc_override_param").value)
        self._require_rc_offb = bool(self.get_parameter("require_rc_offboard").value)    # NEW
        self._rc_mode_ch      = int(self.get_parameter("rc_mode_channel").value)         # NEW
        self._rc_offb_pwm     = int(self.get_parameter("rc_offboard_pwm").value)         # NEW
        self._rc_offb_tol     = int(self.get_parameter("rc_offboard_tol").value)         # NEW
        self._rc_offb_to      = float(self.get_parameter("rc_offboard_timeout").value)   # NEW
        self._rc_channels     = []                                                       # NEW
        self._dry_run     = bool(self.get_parameter("dry_run").value)
        self._mission_to  = float(self.get_parameter("mission_timeout_s").value)
        self._settle_s    = float(self.get_parameter("hover_settle_s").value)
        self._descent_v   = float(self.get_parameter("descent_speed").value)
        self._land_handoff = float(self.get_parameter("land_handoff_alt").value)
        self._auto_land   = bool(self.get_parameter("auto_land_mode").value)
        self._write_px4   = bool(self.get_parameter("write_px4_params").value)
        self._ekf2_hgt    = int(self.get_parameter("ekf2_hgt_ref").value)
        self._ekf_pre_wait = float(self.get_parameter("ekf_pre_wait_s").value)
        self._status_period = float(self.get_parameter("status_period_s").value)
        self._color       = bool(self.get_parameter("color_output").value)
        self._warn_overrun = bool(self.get_parameter("warn_replan_overrun").value)
        # NEW (2026-09-15): yaw while FLYING. "home" = hold the home yaw
        # (nose toward the goal); "velocity" = old behaviour (base code).
        self.declare_parameter("flying_yaw_mode", "home")
        self._flying_yaw_mode = str(
            self.get_parameter("flying_yaw_mode").value).strip().lower()
        if self._flying_yaw_mode not in ("home", "velocity", "smooth"):
            self.get_logger().warn(
                f"flying_yaw_mode '{self._flying_yaw_mode}' unknown -> 'home'")
            self._flying_yaw_mode = "home"
        # NEW (2026-09-15, YAWSMOOTH): "smooth" = nose follows the trajectory
        # direction, limited so it cannot swing like the old "velocity" mode.
        self.declare_parameter("yaw_smooth_tau_s", 1.0)
        self.declare_parameter("yaw_rate_max_dps", 30.0)
        self.declare_parameter("yaw_min_speed", 0.15)
        self.declare_parameter("yaw_max_offset_deg", 60.0)
        self.declare_parameter("yaw_deadband_deg", 15.0)
        self._yaw_tau = float(np.clip(
            float(self.get_parameter("yaw_smooth_tau_s").value), 0.0, 5.0))
        self._yaw_rate_max = math.radians(float(np.clip(
            float(self.get_parameter("yaw_rate_max_dps").value), 5.0, 90.0)))
        self._yaw_min_speed = float(np.clip(
            float(self.get_parameter("yaw_min_speed").value), 0.05, 1.0))
        self._yaw_max_off = math.radians(float(np.clip(
            float(self.get_parameter("yaw_max_offset_deg").value), 0.0, 90.0)))
        self._yaw_deadband = math.radians(float(np.clip(
            float(self.get_parameter("yaw_deadband_deg").value), 0.0, 45.0)))
        self._pub_sp = _FlyingYawLockPublisher(self._pub_sp, self)

        # ── Safety state ─────────────────────────────────────────────────────
        self._rc_override = False
        self._link_lost   = False
        self._in_offboard_mission = False
        self._stream_on   = True
        self._prev_mode   = ""
        # NEW (2026-09-10): kill switch + unexpected disarm
        self._kill_en      = bool(self.get_parameter("kill_switch_enabled").value)
        self._kill_ch      = int(self.get_parameter("kill_channel").value)
        self._kill_on_pwm  = int(self.get_parameter("kill_on_pwm").value)
        self._kill_pwm     = None    # last raw value on the kill channel
        self._kill_hits    = 0       # consecutive "engaged" samples
        self._kill_now     = False   # live, debounced switch state
        self._kill_watch   = False   # latching enabled from the ARM step on
        self._kill_latched = False   # engaged after that -> terminal
        self._disarm_abort = False   # disarm this node did not request
        self._prev_conn   = False
        self._have_pose   = False

        self._home_locked = False
        self._home_xy     = np.zeros(2)
        self._home_yaw    = 0.0
        self._yaw_lock_ref = 0.0     # NEW (2026-09-16, RTHYAW)
        self._ground_z    = 0.0
        self._hold_z      = 0.0      # z held during IDLE/TAKEOFF/LANDING

        self._batt_v   = None
        self._batt_pct = None
        # GPS quality from /px4/sensors ("gps_quality" block). None = the
        # reader has not sent one yet, which the pre-arm gate treats as
        # "not proven good" rather than as "fine".
        self._gps_q    = None

        # ── Phase / monitoring state ─────────────────────────────────────────
        self._phase_idx    = 0
        self._phase_name   = "PREFLIGHT"
        self._t_run_start  = time.time()   # T+ shown on every banner/status
        self._t_phase      = time.time()
        self._last_replan_ms = 0.0         # filled by the _replan() wrapper
        self._replan_slow_n  = 0           # how many overran the budget


        # NEW: raw RC channels for _check_rc_offboard_position(). MAVROS
        # publishes this BEST_EFFORT, so the QoS has to match or nothing
        # ever arrives.
        self.create_subscription(
            RCIn, "/mavros/rc/in", self._cb_rc_in,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=5,
                       durability=DurabilityPolicy.VOLATILE))

        # Watchdog separate from the mission loop: the mission loop can be
        # blocked inside _replan (can take hundreds of ms), while geofence/
        # battery violations must be detected on a steady cadence.
        self._wd_timer = self.create_timer(0.2, self._watchdog)

        # Replace the base's 5 s status timer: 5 s is too coarse to monitor a
        # flight by. The callback itself is this class's _print_status().
        try:
            self._status_timer.cancel()
        except Exception:
            pass
        self._status_timer = self.create_timer(
            self._status_period, self._print_status)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  MODE: REAL DRONE (fm_inference_real_node)")
        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"  Goal          : {self._goal_dist:.1f} m ahead, "
            f"{self._goal_lat:+.1f} m lateral "
            + ("(HOME frame)" if self._use_home_frame else "(absolute ODOM frame)"))
        self.get_logger().info(
            f"  Geofence      : fwd {self._fence_fwd:.1f} / back "
            f"{self._fence_back:.1f} / lat +-{self._fence_lat:.1f} m "
            f"| abort radius {self._max_home_d:.1f} m")
        self.get_logger().info(
            f"  RC override   : {'ACTIVE' if self._rc_ovr_en else 'DISABLED'}"
            f" | param verification: {self._verify_rc}")
        self.get_logger().info(
            f"  Battery       : "
            + (f"min {self._min_batt_v:.1f} V " if self._min_batt_v > 0 else "")
            + (f"min {self._min_batt_p:.0f} %" if self._min_batt_p > 0 else "")
            + ("disabled" if self._min_batt_v <= 0 and self._min_batt_p <= 0 else ""))
        # NEW (2026-09-15)
        if self._flying_yaw_mode == "home":
            yaw_txt = "HOME yaw locked (nose toward goal)"
        elif self._flying_yaw_mode == "smooth":   # NEW (2026-09-15, YAWSMOOTH)
            yaw_txt = (
                f"SMOOTH follow path | tau {self._yaw_tau:.1f} s | "
                f"<= {math.degrees(self._yaw_rate_max):.0f} deg/s | "
                f"+-{math.degrees(self._yaw_max_off):.0f} deg of goal dir | "
                f"deadband {math.degrees(self._yaw_deadband):.0f} deg | "
                f"min speed {self._yaw_min_speed:.2f} m/s")
        else:
            yaw_txt = "follows trajectory velocity (old behaviour)"
        self.get_logger().info("  Flying yaw    : " + yaw_txt)
        if self._require_gps:
            self.get_logger().info(
                f"  GPS gate      : ENABLED — fix>=3D, sats>={self._min_sats}, "
                f"HDOP<={self._max_hdop:.1f} (skipped in dry_run)")
        else:
            self.get_logger().warn(
                "  GPS gate      : DISABLED (require_gps:=false) — indoor/VIO only!")
        if self._dry_run:
            self.get_logger().warn(
                "  DRY RUN       : NOT arming, NOT sending setpoints. "
                "Full pipeline still runs (safe for bench testing).")
        self.get_logger().info("=" * 62)

    # ── Terminal monitoring: phase banners + rich status line ─────────────────
    #
    # Everything below is HARDWARE-ONLY presentation. It never changes a
    # decision, a threshold, or a setpoint — fm_inference_base.py stays
    # byte-identical to the simulation pipeline (see README section 1).

    def _c(self, code: str) -> str:
        """ANSI escape, or '' when colour is disabled."""
        # NEW (2026-09-14): getattr — the base __init__ calls _c() for the
        # Backend banner line before this class sets self._color
        # (AttributeError crashed the node at startup). Default: colour on.
        return _ANSI.get(code, "") if getattr(self, "_color", True) else ""

    def _elapsed(self) -> str:
        s = int(time.time() - self._t_run_start)
        return f"T+{s // 60:02d}:{s % 60:02d}"

    def _emit(self, text: str, warn: bool = False):
        """Log at INFO or WARN from two SEPARATE call sites.

        rclpy caches logger state per call site and raises
        'Logger severity cannot be changed between calls' if the same site is
        used at two severities. Selecting the method into a variable and
        calling it from one line does exactly that — it crashed the node the
        first time a WARN phase (ARMING) followed an INFO phase (PREFLIGHT).
        Keep these two calls on their own lines."""
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _announce_phase(self, name: str, detail: str = "", warn: bool = False):
        """Unmissable banner on every mission-phase transition.

        The operator is watching this terminal while a pilot holds the RC.
        Which phase the drone is in must be readable at a glance, which a
        stream of same-looking INFO lines does not give you."""
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
        """Replaces the base status line with a flight-monitoring one.

        Shows PHYSICAL altitude (z - ground_z) rather than raw local-frame z:
        raw z is meaningless to an operator standing next to the drone, and
        reading it wrong is exactly what went unnoticed on 2026-08-27."""
        pos   = self._drone_state.global_pos
        speed = float(np.linalg.norm(self._drone_state.global_vel[:2]))
        col   = self._c(_PHASE_COLOR.get(self._phase_name, "white"))
        rst   = self._c("reset")
        bold  = self._c("bold")

        alt = float(pos[2]) - self._ground_z
        parts = [f"alt={alt:5.2f}m"]

        if self._home_locked:
            d_home = float(np.linalg.norm(pos[:2] - self._home_xy))
            fence = ""
            if self._max_home_d > 0.0:
                frac = d_home / self._max_home_d
                if frac > 0.8:
                    fence = self._c("red") + "!" + rst
                elif frac > 0.6:
                    fence = self._c("yellow") + "." + rst
            parts.append(f"home={d_home:5.2f}m{fence}")

        if self._global_target is not None:
            d_goal = float(np.linalg.norm(pos[:2] - self._global_target))
            parts.append(f"goal={d_goal:5.2f}m")

        parts.append(f"spd={speed:4.2f}")

        if self._esdf.is_ready():
            # +0.30 = BODY_RADIUS_M: get_edt_dis is a body-EDGE distance, but
            # every threshold the operator is comparing against (guard_clear,
            # safe_dis) is a CENTRE-OF-MASS distance. Show the same frame.
            clear = float(self._esdf.get_edt_dis(pos[:2])) + 0.30
            if clear < self._guard_clear:
                parts.append(f"clear={self._c('red')}{clear:4.2f}m{rst}")
            elif clear < self._guard_clear + 0.2:
                parts.append(f"clear={self._c('yellow')}{clear:4.2f}m{rst}")
            else:
                parts.append(f"clear={clear:4.2f}m")
        else:
            parts.append(f"clear={self._c('yellow')}no-map{rst}")

        if self._batt_v is not None:
            parts.append(f"bat={self._batt_v:4.1f}V")
        if self._kill_now:  # NEW (2026-09-10)
            parts.append(f"{self._c('red')}KILL{rst}")

        # What the drone is actually DOING right now, which "FLYING" alone
        # does not tell you: a guard that invalidated the trajectory leaves
        # the state at FLYING while the drone just sits there.
        act = ""
        if self._mission_state == self.STATE_FLYING:
            if self._escape_active:
                act = f" {self._c('red')}{bold}[ESCAPE]{rst}"
            else:
                with self._traj_lock:
                    moving = self._traj.is_valid()
                act = ("" if moving
                       else f" {self._c('yellow')}[HOLDING - no trajectory]{rst}")

        inf = np.mean(self._inference_times) if self._inference_times else 0.0
        tail = (f"replan#{self._replan_count} {self._last_replan_ms:.0f}ms "
                f"(inf {inf:.0f}ms)")
        if self._replan_slow_n:
            tail += f" {self._c('yellow')}slow x{self._replan_slow_n}{rst}"

        self.get_logger().info(
            f"{col}{bold}[{self._elapsed()}] {self._phase_name:<9s}{rst}"
            f"{act} | " + " | ".join(parts) + f" | {self._c('dim')}{tail}{rst}")

    def _replan(self):
        """Times the base replan so an overrun is visible in the terminal.

        planning_time_ahead is the compute-time budget: the trajectory starts
        at the drone's predicted position that far ahead, but is played back
        from the moment it is installed. A replan that takes longer installs a
        trajectory whose start point the drone has ALREADY passed, and PX4
        then chases a setpoint behind itself."""
        t0 = time.time()
        super()._replan()
        dt = time.time() - t0
        self._last_replan_ms = dt * 1000.0
        if self._warn_overrun and dt > self._dt_ahead:
            self._replan_slow_n += 1
            self.get_logger().warn(
                f"{self._c('yellow')}[SLOW] Replan {dt * 1000:.0f}ms > budget "
                f"{self._dt_ahead * 1000:.0f}ms (planning_time_ahead) — "
                f"trajectory starts {(dt - self._dt_ahead):.2f}s behind the "
                f"drone{self._c('reset')}", throttle_duration_sec=3.0)

    # ── Callback: detect RC override & link loss ──────────────────────────────

    def _cb_state(self, msg):
        prev_armed = getattr(self, "_armed", False)   # NEW (2026-09-10)
        super()._cb_state(msg)   # fills in _connected / _armed / _mode
        try:
            # NEW (2026-09-10): a disarm during the mission that this node did
            # not request ends it. _in_offboard_mission is cleared before the
            # AUTO.LAND hand-off, where a disarm is expected.
            if (self._in_offboard_mission and prev_armed and not self._armed
                    and not self._disarm_abort):
                self._disarm_abort = True
                self._stream_on = False
                self.get_logger().error(
                    "[ARM] UNEXPECTED DISARM during active mission — setpoints stopped.")
            if (self._in_offboard_mission and self._prev_conn
                    and not self._connected and not self._link_lost):
                self._link_lost = True
                self.get_logger().error(
                    "[LINK] FCU DISCONNECTED during active mission — setpoints stopped.")
            # Unexpected mode change during the mission = the pilot took over.
            # AUTO.LAND is NOT excluded any more (2026-09-09): every place this
            # node sets AUTO.LAND itself clears _in_offboard_mission FIRST, so
            # the guard above already covers that. Excluding the mode as well
            # meant a pilot flicking the switch to Land went undetected.
            if (self._rc_ovr_en and self._in_offboard_mission
                    and self._prev_mode == "OFFBOARD"
                    and self._mode not in ("OFFBOARD", "")
                    and not self._rc_override):
                self._rc_override = True
                self.get_logger().error(
                    f"[RC] MODE CHANGED: OFFBOARD -> {self._mode}")
                self.get_logger().error(
                    "[RC] PILOT TOOK OVER — setpoints stopped entirely.")
            self._prev_mode = self._mode
            self._prev_conn = self._connected
        except Exception:
            pass

    def _cb_sensors(self, msg):
        super()._cb_sensors(msg)   # position/velocity/yaw -> _drone_state
        try:
            import json
            d = json.loads(msg.data)
            # IMPORTANT: don't use "position != (0,0,0)" as the pose-ready
            # signal. PX4's EKF genuinely STARTS at exactly (0,0,0) when it
            # initializes at home, so a check like that could wait forever.
            # The correct check: whether the reader is actually sending the
            # local_x field.
            if "local_x" in d:
                self._have_pose = True
            b = d.get("battery")
            if isinstance(b, dict):
                v = b.get("voltage_V")
                if v is not None and not math.isnan(float(v)):
                    self._batt_v = float(v)
            p = d.get("battery_pct")
            if p is not None:
                self._batt_pct = float(p)
            q = d.get("gps_quality")
            if isinstance(q, dict):
                self._gps_q = q
        except Exception:
            pass

    # ── Setpoint: safety gate + hold home while climbing/descending ──────────

    def _publish_cmd(self):
        """Single gate for ALL setpoints. Closes instantly on RC override /
        link loss / dry run, without waiting for the mission loop to notice
        (the ~0.1s gap that takeoff_land_node deliberately closes too)."""
        if self._dry_run or self._rc_override or self._link_lost \
                or self._kill_latched or self._disarm_abort \
                or not self._stream_on:
            return
        # While not (yet) in the FLYING phase, hold XY HOME — not the
        # instantaneous position. Using the instantaneous position would
        # turn every bit of EKF drift into a command.
        if (self._home_locked and self._mission_state != self.STATE_FLYING):
            msg = PositionTarget()
            msg.header.stamp     = self.get_clock().now().to_msg()
            msg.header.frame_id  = "map"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
            msg.position.x = float(self._home_xy[0])
            msg.position.y = float(self._home_xy[1])
            msg.position.z = float(self._hold_z)
            msg.yaw        = float(self._home_yaw)
            self._pub_sp.publish(msg)
            return
        super()._publish_cmd()

    # ── Safety watchdog (5 Hz) ─────────────────────────────────────────────────

    def _watchdog(self):
        if self._mission_state not in (self.STATE_TAKEOFF, self.STATE_FLYING):
            return
        if self._abort_reason is not None:
            return
        pos = self._drone_state.global_pos

        if self._home_locked and self._max_home_d > 0.0:
            d = float(np.linalg.norm(pos[:2] - self._home_xy))
            if d > self._max_home_d:
                self._abort_reason = (
                    f"GEOFENCE: {d:.1f} m from home (limit {self._max_home_d:.1f} m)")
                return

        if self._cruise_z is not None and self._max_alt_err > 0.0 \
                and self._mission_state == self.STATE_FLYING:
            dz = abs(float(pos[2]) - float(self._cruise_z))
            if dz > self._max_alt_err:
                self._abort_reason = (
                    f"ALTITUDE deviated {dz:.2f} m from cruise "
                    f"(limit {self._max_alt_err:.2f} m)")
                return

        if self._min_batt_v > 0.0 and self._batt_v is not None \
                and self._batt_v < self._min_batt_v:
            self._abort_reason = (
                f"BATTERY {self._batt_v:.2f} V < {self._min_batt_v:.2f} V")
            return
        if self._min_batt_p > 0.0 and self._batt_pct is not None \
                and 0.0 <= self._batt_pct < self._min_batt_p:
            self._abort_reason = (
                f"BATTERY {self._batt_pct:.0f}% < {self._min_batt_p:.0f}%")
            return

        if self._in_offboard_mission and not self._armed:
            self._abort_reason = "Unexpected DISARM while flying"

    # ── PX4 parameter verification (from takeoff_land_node) ───────────────────

    def _get_px4_param_int(self, name, timeout=5.0):
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
        client = self.create_client(ParamGet, "/mavros/param/get")
        if client.wait_for_service(timeout_sec=2.0):
            req = ParamGet.Request()
            req.param_id = name
            res = self._call_srv(client, req, timeout=timeout)
            if res and res.success:
                return int(res.value.integer or res.value.real)
        return None

    def _check_rc_override_param(self) -> bool:
        """COM_RC_OVERRIDE must be 2 or 3, otherwise the RC stick can
        PHYSICALLY NOT take over OFFBOARD — the detection in _cb_state
        would never help."""
        if not self._rc_ovr_en or not self._verify_rc:
            return True
        pull = self.create_client(ParamPull, "/mavros/param/pull")
        if pull.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("[SAFETY] Syncing PX4 parameters...")
            self._call_srv(pull, ParamPull.Request(force_pull=False), timeout=60.0)
        value, t0 = None, time.time()
        while value is None and rclpy.ok() and time.time() - t0 < 30.0:
            value = self._get_px4_param_int("COM_RC_OVERRIDE")
            if value is None:
                time.sleep(3.0)
        if value is None:
            self.get_logger().warn(
                "[SAFETY] COM_RC_OVERRIDE could not be read — continuing, but "
                "VERIFY MANUALLY in QGroundControl.")
            return True
        if value not in (2, 3):
            self.get_logger().error("=" * 62)
            self.get_logger().error(
                f"[SAFETY] COM_RC_OVERRIDE={value} — the RC stick CANNOT "
                "take over OFFBOARD!")
            self.get_logger().error(
                "[SAFETY] Set it to 2 in QGroundControl, then reboot PX4.")
            self.get_logger().error("=" * 62)
            return False
        self.get_logger().info(f"[SAFETY] COM_RC_OVERRIDE={value} — OK.")
        obl = self._get_px4_param_int("COM_OBL_RC_ACT", timeout=3.0)
        if obl is not None:
            self.get_logger().info(
                f"[SAFETY] COM_OBL_RC_ACT={obl} (failsafe action if the setpoint "
                "stream stops — make sure it's Hold/Land/Return in QGC).")
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
        cuts the motors. Once _kill_watch is set (ARM step), the first engaged
        state is latched for the rest of this process."""
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
                "setpoints stopped. Only LAND + DISARM will be requested.")

    def _kill_abort(self):
        """Terminal reaction to the kill switch (pilot's decision, 2026-09-10):
        setpoints are already stopped; ask for AUTO.LAND so that a revert
        inside PX4's 5 s kill window resumes in LAND (PX4's offboard-loss
        failsafe would get there too, after COM_OF_LOSS_T); keep requesting a
        normal DISARM, which PX4 only accepts once landed; exit without ever
        arming or entering OFFBOARD again."""
        self._stream_on = False
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
        self._mission_state = self.STATE_DONE
        self._shutdown()

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
        an unknown fix is exactly the situation that put a drone in a tree on
        2026-08-27, so it must never be treated as acceptable."""
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
            if self._aborted() or self._abort_reason is not None:
                self.get_logger().warn("[GPS] Gate cancelled (abort).")
                return False
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
            "a drone drifted into a tree on 2026-08-27.")
        self.get_logger().error(
            "[GPS] Move to open sky away from trees/buildings and retry, or "
            "set require_gps:=false ONLY if a healthy VIO/optical-flow source "
            "is providing PX4's local position.")
        self.get_logger().error("=" * 62)
        return False

    # ── NEW (2026-09-13): octomap band relative to the ground ─────────────────

    def _configure_octomap_band(self, alt_above_ground):
        """Occupancy band as HEIGHT ABOVE THE GROUND, shifted into odom z.

        The parent computes the band in absolute odom z as
        [max(0.35, z - 0.7), z + 1.0] and is called with target_alt. That is
        only right when ground_z is ~0. On 2026-09-13 (height reference =
        GPS) the drone sat at ground_z = -6.16 m, so it cruised at z = -4.16
        while the band stayed at [1.30, 3.00] — 5-7 m above the drone. The
        octomap never saw anything at flight height, every cell around the
        drone stayed unknown, the guard reported "drone is OFF-MAP" from the
        first FLYING tick, and the blind abort landed it after 10 s.

        The parent (fm_inference_base.py) is left untouched on purpose — it is
        shared with simulation, where ground_z is ~0 and the result is the
        same. Here the same band is computed relative to the ground (the 0.35
        m floor keeps ground returns out) and then shifted by ground_z.
        """
        occ_min = self._ground_z + max(0.35, alt_above_ground - 0.7)
        occ_max = self._ground_z + alt_above_ground + 1.0
        if not self._octo_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"[REAL] octomap set_parameters absent — set manually: "
                f"occ_min_z~{occ_min:.2f} occ_max_z~{occ_max:.2f}")
            return
        req = SetParameters.Request()
        for name, val in (("occupancy_min_z", occ_min),
                          ("occupancy_max_z", occ_max)):
            p = RclParameter()
            p.name = name
            pv = RclParameterValue()
            pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(val)
            p.value = pv
            req.parameters.append(p)
        res = self._call_srv(self._octo_param_client, req)
        ok  = bool(res and all(r.successful for r in res.results))
        self.get_logger().info(
            f"[REAL] {'ok' if ok else 'FAILED'} Octomap band "
            f"[{occ_min:.2f}, {occ_max:.2f}] odom z = "
            f"[{occ_min - self._ground_z:.2f}, {occ_max - self._ground_z:.2f}] m "
            f"above ground (ground_z={self._ground_z:.2f})")
        if ok and self._octo_reset_client.wait_for_service(timeout_sec=3.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(),
                           timeout=5.0)

    # ── Home frame: goal + geofence ───────────────────────────────────────────

    def _lock_home(self):
        self._home_xy  = self._drone_state.global_pos[:2].copy()
        self._home_yaw = float(self._drone_state.yaw)
        self._home_locked = True

        if self._use_home_frame:
            c, s = math.cos(self._home_yaw), math.sin(self._home_yaw)
            fwd = np.array([c, s])            # drone's nose direction at home
            lat = np.array([-s, c])           # left
            self._global_target = (self._home_xy
                                   + fwd * self._goal_dist
                                   + lat * self._goal_lat)
            # The geofence is defined in the home frame, then wrapped into
            # an axis-aligned box (AABB) because the ESDF only understands
            # axis-aligned boxes. The AABB is a SUPERSET of the rotated box,
            # so it's always more permissive — the hard safeguard remains
            # max_home_dist in the watchdog.
            corners = []
            for f in (-self._fence_back, self._fence_fwd):
                for l in (-self._fence_lat, self._fence_lat):
                    corners.append(self._home_xy + fwd * f + lat * l)
            corners = np.array(corners)
            self._arena_x_min = float(corners[:, 0].min())
            self._arena_x_max = float(corners[:, 0].max())
            self._arena_y_min = float(corners[:, 1].min())
            self._arena_y_max = float(corners[:, 1].max())
        else:
            self._global_target = np.array([self._goal_x, 0.0])

        self._arena_center = np.array([
            (self._arena_x_min + self._arena_x_max) / 2.0,
            (self._arena_y_min + self._arena_y_max) / 2.0])
        # The virtual ESDF wall is moved along with the newly computed box.
        self._esdf.arena_bounds = (self._arena_x_min, self._arena_x_max,
                                   self._arena_y_min, self._arena_y_max)

        self.get_logger().info(
            f"[REAL] home=({self._home_xy[0]:.2f},{self._home_xy[1]:.2f}) "
            f"yaw={math.degrees(self._home_yaw):.0f}deg ground_z={self._ground_z:.2f} m")
        self.get_logger().info(
            f"[REAL] goal=({self._global_target[0]:.2f},"
            f"{self._global_target[1]:.2f})  arena x[{self._arena_x_min:.1f},"
            f"{self._arena_x_max:.1f}] y[{self._arena_y_min:.1f},"
            f"{self._arena_y_max:.1f}]")

    # ── Hard abort: stop sending setpoints ────────────────────────────────────

    def _hard_stop(self, reason, try_auto_land=False):
        if self._kill_latched:  # NEW (2026-09-10): the kill switch has its own terminal path
            return self._kill_abort()
        if self._disarm_abort:  # NEW (2026-09-10): never command a mode for a disarm
            reason, try_auto_land = f"UNEXPECTED DISARM ({reason})", False
        self._stream_on = False
        self._in_offboard_mission = False
        self._announce_phase("ABORT", f"MISSION STOPPED: {reason}", warn=True)
        self.get_logger().error("=" * 62)
        self.get_logger().error(f"[REAL] MISSION STOPPED: {reason}")
        if try_auto_land:
            self.get_logger().error("[REAL] Handing off to AUTO.LAND.")
            try:
                self._set_mode("AUTO.LAND")
            except Exception:
                pass
        else:
            self.get_logger().error(
                "[REAL] Setpoints stopped. Control is with the pilot / PX4 failsafe.")
        self.get_logger().error("=" * 62)
        self._mission_state = self.STATE_DONE

    def _aborted(self) -> bool:
        """True if some condition requires the mission to stop right now."""
        return (self._rc_override or self._link_lost
                or self._kill_latched or self._disarm_abort)  # NEW (2026-09-10)

    # ── Wait for altitude with hardware checks ─────────────────────────────────

    def _wait_altitude_real(self, target_z, tol=0.15, timeout=30.0):
        t0, stable = time.time(), None
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._aborted() or self._abort_reason is not None:
                return False
            z  = float(self._drone_state.global_pos[2])
            vz = abs(float(self._drone_state.global_vel[2]))
            if abs(z - target_z) < tol and vz < 0.2:
                stable = stable or time.time()
                if time.time() - stable >= 2.0:
                    return True
            else:
                stable = None
            time.sleep(0.1)
        return False

    # ── EKF pre-wait (from takeoff_land_node) ──────────────────────────────────

    def _wait_ekf_stable(self, tol: float = 0.08, stable_dur: float = 3.0,
                         timeout: float = 45.0):
        """Determine ground_z, or return None if the estimate cannot be trusted.

        This deliberately does NOT call super(). fm_inference_base's version
        is kept byte-identical to the simulation pipeline, and it has the flaw
        that caused the 2026-08-27 crash: it takes the standard deviation of
        the last TEN samples — a ONE SECOND window — and accepts whatever mean
        that window holds. On the ground that day the EKF z was swinging
        roughly +-5 m over ~50 s; at the turning points of that slow swing the
        one-second spread is tiny, so it declared "stable" with ground_z =
        5.459 m for a drone sitting on the ground, and the takeoff that
        followed chased a phantom position error sideways into a tree.

        Changing the base file would break sim/real parity for the PLANNER,
        which is the one thing this package promises not to touch. Ground
        truth for takeoff is a hardware pre-flight concern, not planner logic,
        and this node already owned an override of it — so the hardware-grade
        version lives here.

        Three independent criteria must hold CONTINUOUSLY for stable_dur:
          a) std over the window < tol            — fast jitter
          b) peak-to-peak <= max_ground_drift     — the SLOW drift a short
                                                    std window cannot see
          c) |mean z| <= max_ground_z             — we take off FROM THE
                                                    GROUND, so z far from 0 is
                                                    proof the estimate is
                                                    broken however steady it
                                                    looks. Steadiness alone
                                                    never establishes
                                                    correctness.
        Returns None on timeout — "could not confirm the EKF, so fly on it
        anyway" is exactly backwards for a safety check.
        """
        self.get_logger().info(
            f"[REAL] Waiting {self._ekf_pre_wait:.0f}s for initial GPS/EKF convergence...")
        t_pre = time.time()
        while rclpy.ok() and time.time() - t_pre < self._ekf_pre_wait:
            if self._aborted() or self._abort_reason is not None:
                self.get_logger().warn("[REAL] EKF pre-wait cancelled (abort).")
                return None
            time.sleep(0.1)

        n_win = max(10, int(self._ekf_window_s / 0.1))
        self.get_logger().info(
            f"[REAL] Waiting for EKF Z: std<{tol}m, spread<={self._max_gnd_drift}m "
            f"over {self._ekf_window_s:.1f}s, |z|<={self._max_ground_z}m, "
            f"held {stable_dur:.0f}s (timeout {timeout:.0f}s)...")

        t0 = time.time()
        z_hist = []
        stable_start = None
        last_report = 0.0
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._aborted() or self._abort_reason is not None:
                self.get_logger().warn("[REAL] EKF wait cancelled (abort).")
                return None
            z_hist.append(float(self._drone_state.global_pos[2]))
            if len(z_hist) > n_win:
                z_hist.pop(0)

            problem = None
            if len(z_hist) < n_win:
                problem = f"collecting samples ({len(z_hist)}/{n_win})"
            else:
                win    = z_hist[-n_win:]
                z_std  = float(np.std(win))
                z_pp   = float(max(win) - min(win))
                z_mean = float(np.mean(win))
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
                        f"[REAL] EKF stable. ground_z={z_ground:.3f}m "
                        f"(std={float(np.std(win)):.4f}, "
                        f"spread={float(max(win) - min(win)):.3f}m)")
                    return z_ground
            else:
                stable_start = None
                if now - last_report >= 5.0:
                    last_report = now
                    self.get_logger().warn(
                        f"[REAL] EKF not usable yet: {problem} "
                        f"({timeout - (now - t0):.0f}s left)")
            time.sleep(0.1)

        self.get_logger().error("=" * 62)
        self.get_logger().error(
            "[REAL] EKF NEVER BECAME TRUSTWORTHY within the timeout.")
        if z_hist:
            win = z_hist[-min(len(z_hist), n_win):]
            self.get_logger().error(
                f"[REAL] Last window: z={float(np.mean(win)):.2f}m, "
                f"spread={float(max(win) - min(win)):.2f}m, "
                f"std={float(np.std(win)):.3f}m")
        self.get_logger().error(
            "[REAL] REFUSING TO FLY on an unverified position estimate — this "
            "is what put a drone in a tree on 2026-08-27.")
        self.get_logger().error("=" * 62)
        return None

    # ── Controlled landing (from takeoff_land_node) ────────────────────────────

    def _controlled_descent(self, from_z):
        self._mission_state = self.STATE_LANDING
        self._cruise_z = None
        # Land IN PLACE: returning home along a route that may not be
        # obstacle-free is far riskier than descending at the current position.
        self._home_xy = self._drone_state.global_pos[:2].copy()
        dt     = 1.0 / self._cmd_hz
        step   = self._descent_v * dt
        target = self._ground_z + self._land_handoff
        z = from_z
        self.get_logger().info(
            f"[REAL] LANDING at ({self._home_xy[0]:.2f},{self._home_xy[1]:.2f}) "
            f"-> z={target:.2f} m")
        while rclpy.ok() and z > target:
            if self._aborted():
                return
            z = max(target, z - step)
            self._hold_z = z
            if not self._armed:
                self.get_logger().info("[REAL] Auto-disarm while descending (land detector).")
                return
            time.sleep(dt)
        self.get_logger().info(
            f"[REAL] Descent finished at z={self._drone_state.global_pos[2]:.2f} m")

    # ── Mission sequence ────────────────────────────────────────────────────────

    def run_sequence(self):
        self._t_run_start = time.time()
        self._announce_phase(
            "PREFLIGHT",
            "FCU -> RC param -> pose -> GPS -> depth/ESDF -> ground_z -> home")
        # 1. FCU connection
        self.get_logger().info("[REAL] Waiting for MAVROS (/px4/state)...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > self._conn_timeout:
                self.get_logger().error("[REAL] FCU connection timeout.")
                return self._shutdown()
            time.sleep(0.2)
        self.get_logger().info("[REAL] FCU connected.")

        # 2. Verify COM_RC_OVERRIDE BEFORE anything that could fly
        if not self._dry_run and not self._check_rc_override_param():
            return self._shutdown()

        # 3. Valid local pose (GPS / VIO / optical flow)
        self.get_logger().info("[REAL] Waiting for local position (/px4/sensors)...")
        t0 = time.time()
        while rclpy.ok() and not self._have_pose:
            if time.time() - t0 > 30.0:
                self.get_logger().error(
                    "[REAL] No local position. PX4 needs GPS/VIO/flow "
                    "for position-based OFFBOARD.")
                return self._shutdown()
            time.sleep(0.2)

        # 3b. GPS quality gate — BEFORE ground_z/home are latched, because both
        # are derived from the position estimate we are about to trust for the
        # whole flight. Skipped in dry_run: that path never arms and is meant
        # to be runnable on a bench indoors (stage 1 of the README ladder),
        # where there is no sky view at all. See _wait_gps_quality().
        if not self._dry_run and not self._wait_gps_quality():
            self.get_logger().error("[REAL] Aborting — GPS not safe to fly on.")
            return self._shutdown()

        # 4. Perception ready
        self.get_logger().info("[REAL] Waiting for depth + ESDF (octomap)...")
        t0 = time.time()
        while rclpy.ok() and (self._latest_depth is None
                              or not self._esdf.is_ready()):
            if time.time() - t0 > 90.0:
                self.get_logger().error(
                    "[REAL] Timeout depth/ESDF. Check: camera driver, "
                    "gemini2_depth_bridge_node, TF odom->base_link->"
                    "camera_depth_frame, and octomap_server.")
                return self._shutdown()
            time.sleep(0.5)
        self.get_logger().info("[REAL] Depth + ESDF ready.")

        # 5. PX4 parameters (default: don't touch anything — see docstring)
        if self._write_px4:
            if self._ekf2_hgt >= 0:
                self._set_px4_param_int("EKF2_HGT_REF", self._ekf2_hgt)
            if self._px4_vel_cap > 0.0:
                self._set_px4_param_float("MPC_XY_VEL_MAX", self._px4_vel_cap)
                self._set_px4_param_float("MPC_XY_CRUISE",  self._v_max)
                self._set_px4_param_float("MPC_XY_VEL_ALL", self._px4_vel_cap)
                self.get_logger().info(
                    f"[REAL] PX4 speed cap -> {self._px4_vel_cap:.1f} m/s")
            time.sleep(2.0)
        else:
            self.get_logger().info(
                "[REAL] PX4 parameters NOT changed (write_px4_params:=false). "
                "Set MPC_XY_VEL_MAX / EKF2_HGT_REF via QGC if needed.")

        # 6. ground_z from the EKF
        #
        # The old code here WARNED when |ground_z| > 3.0 and then forced it to
        # 0.0 and flew anyway. That masking was actively dangerous: forcing the
        # number does not fix the estimate it came from. With the EKF reading
        # 8.38 m on the ground (as it did on 2026-08-27), pinning ground_z to
        # 0.0 makes cruise_z = 0 + target_alt, so the drone would be commanded
        # to fly to a z it currently reads as ~7 m BELOW itself — i.e. ordered
        # to descend into the ground. The check now lives inside
        # _wait_ekf_stable() and its verdict is final.
        ground_z = self._wait_ekf_stable()
        if ground_z is None:
            self.get_logger().error(
                "[REAL] Aborting — ground_z could not be established.")
            return self._shutdown()
        self._ground_z = ground_z
        self._cruise_z = self._ground_z + self._alt
        self._hold_z   = self._ground_z

        # 7. Lock home, compute goal & geofence
        self._lock_home()

        # 8. Octomap band + model warm-up
        self._configure_octomap_band(self._alt)
        self.get_logger().info("[REAL] Warming up the model...")
        self._warm_up()

        # ── DRY RUN: full pipeline without flying ────────────────────────────
        if self._dry_run:
            self._announce_phase(
                "DRY RUN",
                "no ARM, no setpoints — full perception + planner only",
                warn=True)
            self.get_logger().warn(
                "[REAL] Watch for '[INF] Replan ok', '[FM] GATE two-sided', and "
                "the /planner/candidates marker in RViz. Ctrl-C to stop.")
            self._mission_state = self.STATE_FLYING
            self._reset_progress()
            last = 0.0
            while rclpy.ok():
                now = time.time()
                if now - last >= self._replan_period:
                    last = now
                    self._replan()
                time.sleep(0.05)
            return

        # 9. Warm up the setpoint stream (PX4 refuses OFFBOARD without a stream)
        self._announce_phase(
            "ARMING",
            f"PROPELLERS WILL SPIN — cruise z={self._cruise_z:.2f} m "
            f"(physical ~{self._alt:.1f} m). Pilot: RC ready.",
            warn=True)
        self._phase_note("warming up the setpoint stream (~2 s)...")
        time.sleep(2.0)

        # 9b. NEW: the pilot's mode switch must already be in the OFFBOARD
        # slot, so that every other switch position stays a live escape route.
        # Skipped in dry_run, which never arms anyway.
        if not self._dry_run and not self._check_rc_offboard_position():
            return self._shutdown()

        # 9c. NEW (2026-09-10): never arm with the kill switch engaged; from
        # here on any engagement of it is terminal (see _kill_abort).
        if not self._check_kill_switch_released():
            return self._shutdown()
        self._kill_watch = True

        # 10. ARM
        self._phase_note("sending ARM...")
        t0, armed = time.time(), False
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
            self.get_logger().error("[REAL] ARM failed.")
            return self._shutdown()

        # 11. OFFBOARD
        if not self._set_mode("OFFBOARD"):
            self._arm(False)
            return self._shutdown()
        t0 = time.time()
        while rclpy.ok() and self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            if self._kill_latched:  # NEW (2026-09-10)
                return self._kill_abort()
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._kill_latched:  # NEW (2026-09-10)
            return self._kill_abort()
        if self._mode != "OFFBOARD":
            self.get_logger().error("[REAL] PX4 did not enter OFFBOARD.")
            self._arm(False)
            return self._shutdown()
        self._in_offboard_mission = True
        if not self._armed:  # NEW (2026-09-10): disarmed before takeoff began
            return self._hard_stop("UNEXPECTED DISARM before takeoff")

        # 12. TAKEOFF
        self._mission_state = self.STATE_TAKEOFF
        self._hold_z = self._cruise_z
        self._announce_phase(
            "TAKEOFF",
            f"climbing to z={self._cruise_z:.2f} m "
            f"(physical ~{self._alt:.1f} m), holding XY home. "
            f"Stable = +-0.15 m and |vz|<0.2 m/s held 2 s.")
        stable = self._wait_altitude_real(self._cruise_z)
        if self._aborted():
            return self._hard_stop("RC override / link loss during takeoff")
        if self._abort_reason is not None:
            return self._land_and_finish()
        if not stable:
            self._phase_note(
                "altitude never became stable within 30 s — proceeding "
                "carefully (mission continues by design)", warn=True)
        else:
            self._phase_note(
                f"reached cruise altitude, alt="
                f"{float(self._drone_state.global_pos[2]) - self._ground_z:.2f} m")

        # 13. Settle + clean map
        # Announced as its own phase: the FSM has no HOVER state, so without
        # this the terminal goes quiet for hover_settle_s + ~1.5 s and an
        # operator cannot tell settling apart from a hang.
        self._announce_phase(
            "HOVER",
            f"settling {self._settle_s:.1f} s over home, then wiping the "
            f"octomap so planning starts from a clean map")
        t_settle = time.time()
        while rclpy.ok() and time.time() - t_settle < self._settle_s:
            if self._aborted():
                return self._hard_stop("RC override / link loss while hovering")
            if self._abort_reason is not None:
                return self._land_and_finish()
            left = self._settle_s - (time.time() - t_settle)
            self.get_logger().info(
                f"{self._c('blue')}[HOVER]{self._c('reset')} "
                f"settling... {left:4.1f} s left",
                throttle_duration_sec=1.0)
            time.sleep(0.1)
        if self._octo_reset_client.wait_for_service(timeout_sec=2.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(), timeout=5.0)
            self._phase_note("octomap wiped — rebuilding a clean map (1.5 s)")
            time.sleep(1.5)
        else:
            self._phase_note("octomap reset service unavailable — map NOT "
                             "wiped, stale ground voxels may remain", warn=True)

        # 14. FLYING — FM replan loop
        # NEW (2026-09-16, RTHREPLAN): the loop moved to _fly_to_target() so it
        # can be run a second time toward home. Behaviour here is unchanged.
        if self._fly_to_target(self._global_target,
                               self._mission_to) == "stopped":
            return

        # 15. LANDING
        self._land_and_finish()

    # NEW (2026-09-16, RTHREPLAN): lifted verbatim out of run_sequence().
    def _fly_to_target(self, target, timeout_s, tag="FLYING",
                       banner_extra="", timeout_label="mission timeout",
                       yaw_ref=None):
        """FM replan loop toward `target` (x, y). Returns:
             "reached" — within 1 m of the target (or the planner said so)
             "abort"   — self._abort_reason set, timeout, or ROS shutdown;
                         the caller lands
             "stopped" — _hard_stop() already ran (RC / link / kill); the
                         caller must return immediately and send nothing
        """
        self._global_target = np.asarray(target, dtype=float)[:2].copy()
        self._reached_target = False
        # NEW (2026-09-16, RTHYAW): yaw reference for this leg.
        self._yaw_lock_ref = (float(self._home_yaw) if yaw_ref is None
                              else float(yaw_ref))
        self._mission_state = self.STATE_FLYING
        self._reset_progress()
        self._announce_phase(
            tag,
            f"goal=({self._global_target[0]:.2f},{self._global_target[1]:.2f}) "
            f"| v_max={self._v_max:.2f} m/s | replan every "
            f"{self._replan_period:.1f} s | RC override is ARMED"
            + banner_extra)
        self._phase_note(
            "expect 'look-ahead guard / UNOBSERVED' hovers at first — the map "
            "was just wiped and has to see the path ahead before advancing")

        t_mission = time.time()
        last_replan = 0.0
        while rclpy.ok():
            now = time.time()
            if self._aborted():
                self._hard_stop("RC override / link loss while flying")
                return "stopped"
            if self._abort_reason is not None:
                self._announce_phase(
                    "ABORT", f"{self._abort_reason} -> landing now", warn=True)
                return "abort"
            if timeout_s > 0.0 and now - t_mission > timeout_s:
                self._announce_phase(
                    "ABORT",
                    f"{timeout_label} {timeout_s:.0f} s reached "
                    f"-> landing now", warn=True)
                return "abort"

            due  = now - last_replan >= self._replan_period
            asap = (self._replan_asap and now - last_replan >= 0.3)
            if due or asap:
                last_replan = now
                self._replan_asap = False
                self._replan()

            dist = float(np.linalg.norm(
                self._drone_state.global_pos[:2] - self._global_target))
            if not self._escape_active and (dist < 1.0 or self._reached_target):
                self._phase_note(
                    f"*** GOAL REACHED *** ({dist:.2f} m remaining) -> landing")
                return "reached"
            time.sleep(0.05)
        return "abort"

    def _land_and_finish(self):
        if self._aborted():
            return self._hard_stop("RC override / link loss")
        self._announce_phase(
            "LANDING",
            f"trajectory dropped, holding XY home, descending at "
            f"{self._descent_v:.2f} m/s to {self._land_handoff:.2f} m "
            f"above ground, then AUTO.LAND")
        with self._traj_lock:
            self._traj.invalidate()
        self._controlled_descent(float(self._drone_state.global_pos[2]))
        if self._aborted():
            return self._hard_stop("RC override / link loss while landing")

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
            self._arm(False)

        self._mission_state = self.STATE_DONE
        self._announce_phase("DONE", "landed and disarmed")
        self.get_logger().info(
            f"[REAL] DONE. Successful replans: {self._replan_count}, "
            f"cost-gate vetoes: {self._n_cost_reject}, "
            f"post-check vetoes: {self._n_graze_reject}")
        if self._gate_total:
            frac = 100.0 * self._gate_two_sided / self._gate_total
            self.get_logger().info(
                f"[REAL] Bimodality gate: {self._gate_two_sided}/"
                f"{self._gate_total} two-sided replans ({frac:.0f}%)")
        if self._abort_reason:
            self.get_logger().error(f"[REAL] (ABORT: {self._abort_reason})")
        self.get_logger().info("=" * 62)
        self._shutdown()

    def _shutdown(self):
        self._stream_on = False
        try:
            self._cmd_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FMInferenceRealNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    seq = threading.Thread(target=node.run_sequence, daemon=True)
    seq.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("[REAL] Ctrl-C — setpoints stopped.")
        node._stream_on = False
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
