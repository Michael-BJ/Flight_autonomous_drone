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

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import ParamGet, ParamPull
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
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

        # ── Safety state ─────────────────────────────────────────────────────
        self._rc_override = False
        self._link_lost   = False
        self._in_offboard_mission = False
        self._stream_on   = True
        self._prev_mode   = ""
        self._prev_conn   = False
        self._have_pose   = False

        self._home_locked = False
        self._home_xy     = np.zeros(2)
        self._home_yaw    = 0.0
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
        return _ANSI.get(code, "") if self._color else ""

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
        super()._cb_state(msg)   # fills in _connected / _armed / _mode
        try:
            if (self._in_offboard_mission and self._prev_conn
                    and not self._connected and not self._link_lost):
                self._link_lost = True
                self.get_logger().error(
                    "[LINK] FCU DISCONNECTED during active mission — setpoints stopped.")
            # Unexpected mode change during the mission = the pilot took over.
            # AUTO.LAND is excluded since we set that one ourselves.
            if (self._rc_ovr_en and self._in_offboard_mission
                    and self._prev_mode == "OFFBOARD"
                    and self._mode not in ("OFFBOARD", "AUTO.LAND", "")
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
        obl = self._get_px4_param_int("COM_OBL_ACT", timeout=3.0)
        if obl is not None:
            self.get_logger().info(
                f"[SAFETY] COM_OBL_ACT={obl} (failsafe action if the setpoint "
                "stream stops — make sure it's Hold/Land/Return in QGC).")
        return True

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
        return self._rc_override or self._link_lost

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

        # 10. ARM
        self._phase_note("sending ARM...")
        t0, armed = time.time(), False
        while rclpy.ok() and time.time() - t0 < self._arm_timeout:
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True
                    break
            time.sleep(1.5)
        if not armed:
            self.get_logger().error("[REAL] ARM failed.")
            return self._shutdown()

        # 11. OFFBOARD
        if not self._set_mode("OFFBOARD"):
            self._arm(False)
            return self._shutdown()
        t0 = time.time()
        while rclpy.ok() and self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._mode != "OFFBOARD":
            self.get_logger().error("[REAL] PX4 did not enter OFFBOARD.")
            self._arm(False)
            return self._shutdown()
        self._in_offboard_mission = True

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
        self._mission_state = self.STATE_FLYING
        self._reset_progress()
        self._announce_phase(
            "FLYING",
            f"goal=({self._global_target[0]:.2f},{self._global_target[1]:.2f}) "
            f"| v_max={self._v_max:.2f} m/s | replan every "
            f"{self._replan_period:.1f} s | RC override is ARMED")
        self._phase_note(
            "expect 'look-ahead guard / UNOBSERVED' hovers at first — the map "
            "was just wiped and has to see the path ahead before advancing")

        t_mission = time.time()
        last_replan = 0.0
        while rclpy.ok():
            now = time.time()
            if self._aborted():
                return self._hard_stop("RC override / link loss while flying")
            if self._abort_reason is not None:
                self._announce_phase(
                    "ABORT", f"{self._abort_reason} -> landing now", warn=True)
                break
            if self._mission_to > 0.0 and now - t_mission > self._mission_to:
                self._announce_phase(
                    "ABORT",
                    f"mission timeout {self._mission_to:.0f} s reached "
                    f"-> landing now", warn=True)
                break

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
                break
            time.sleep(0.05)

        # 15. LANDING
        self._land_and_finish()

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
