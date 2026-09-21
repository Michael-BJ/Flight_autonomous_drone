#!/usr/bin/env python3
"""
recovery.py — RETURN-ALONG-THE-FLOWN-PATH recovery for fm_deploy (2026-09-13)
==================================================================
WHAT HAPPENED
    On 2026-09-13 15:05 fm_deploy took off, hovered and entered FLYING
    normally, then the collision guard reported "drone is OFF-MAP" on every
    tick (an octomap height-band bug, fixed the same day). After 10 s the
    blind abort fired and the node did the only thing it knew: land where
    it was. That flight was still next to home, but the same code path
    lands the drone in the MIDDLE of the course when a recoverable planner
    problem (off-map, stuck, timeout) happens 5-10 m out: on unknown ground,
    possibly next to the obstacle it was trying to avoid, away from the
    pilot.

WHAT THIS ADDS
    A recovery ladder between "planner gave up" and "land":

      1. classify the stop:
           RECOVERABLE  drone off-map / STUCK / mission timeout
                        -> the DRONE is fine, the PLANNER is not
           LAND NOW     GEOFENCE / ALTITUDE / BATTERY / DISARM / barometer
                        -> a vehicle problem: land in place, as before
           pilot / RC override / link loss / kill switch
                        -> untouched, setpoints stop at once, as before
      2. for a recoverable stop: RETURN — retrace the breadcrumb trail the
         drone itself flew (recorded every rth_crumb_spacing m during
         FLYING, home = the take-off point is the first crumb), backwards,
         slowly (rth_speed), at cruise altitude, nose pointing the way it
         moves so the depth camera keeps the octomap current, and land at
         home. Space the drone has flown through is known free space, and
         home is flat, clear and next to the pilot.
      3. every step of the return is guarded, and any failure falls back
         to exactly the old behaviour (land in place):
           - known obstacle within rth_min_clearance of the drone or of the
             carrot (ESDF; virtual arena walls included) -> hold, and land
             in place if still blocked after rth_block_s
           - drone more than rth_max_track_err behind the carrot for
             rth_track_err_s (wind, GPS jump) -> land in place
           - time budget (2x the path at rth_speed + rth_timeout_extra_s)
           - geofence radius, battery limits, unexpected disarm -> as before
           - RC override / link loss / kill switch -> setpoints stop at once
      4. RTH is skipped (plain land-in-place) when the drone is already
         within rth_min_dist of home, when the goal was reached (landing at
         the goal IS the mission), when rth_enabled:=false, or when the
         trail is empty.

    Nothing in the planner (fm_inference_base / fm_inference_node) is
    touched, and fm_inference_real_node's mission sequence is inherited:
    the mixin only overrides _land_and_finish (to try RETURN first) and
    _publish_cmd (to publish the return carrot while in RETURN).

HOW TO USE
    class MyNode(ReturnHomeRecoveryMixin, FMInferenceRealNode):
        def __init__(self):
            super().__init__()
            self._rth_init()
"""
import math
import threading
import time

import numpy as np
import rclpy
from mavros_msgs.msg import PositionTarget

# Same body radius the guards use (fm_inference_base.BODY_RADIUS_M): ESDF
# distances are body-EDGE, the limits below are CENTRE-OF-MASS.
_BODY_RADIUS_M = 0.30


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ReturnHomeRecoveryMixin:

    STATE_RETURN = "RETURN"

    # abort reasons (fm_inference_base / fm_inference_real_node strings)
    _RTH_LAND_NOW = ("GEOFENCE", "ALTITUDE", "BATTERY", "Unexpected DISARM",
                     "BAROMETER")
    _RTH_RECOVERABLE = ("drone off-map", "STUCK")

    # ── setup ─────────────────────────────────────────────────────────────
    def _rth_init(self, log_tag="[RTH]"):
        self._rth_tag = log_tag
        p = self.declare_parameter
        p("rth_enabled",        True)
        p("rth_speed",          0.3)    # m/s along the trail
        p("rth_crumb_spacing",  0.3)    # m between recorded crumbs
        p("rth_min_dist",       1.0)    # m; closer than this to home -> land here
        p("rth_arrive_tol",     0.5)    # m; "at home"
        p("rth_lead_max",       0.6)    # m; carrot never leads by more
        p("rth_max_track_err",  1.5)    # m; drone-to-carrot -> stop
        p("rth_track_err_s",    3.0)    # s the tracking error may persist
        p("rth_min_clearance",  0.50)   # m centre-of-mass; known obstacles
        p("rth_block_s",        8.0)    # s blocked -> land in place
        p("rth_yaw_rate_dps",   45.0)   # deg/s yaw setpoint slew
        p("rth_timeout_extra_s", 30.0)  # s on top of 2x path time
        p("rth_hover_s",        2.0)    # s hover over home before landing
        # NEW (2026-09-16, RTHGOAL): also return home after a SUCCESSFUL mission
        # (goal reached). Default False = unchanged behaviour (land at the goal).
        p("rth_after_goal",     False)
        # NEW (2026-09-16, RTHREPLAN): how to get home.
        #   "trail"  = retrace the breadcrumbs (unchanged, default)
        #   "replan" = run the FM planner again with home as the goal
        p("rth_mode",           "trail")
        p("rth_replan_timeout_s", 0.0)   # 0 = 2 x straight-line time + extra
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._rth_enabled   = bool(g("rth_enabled"))
        self._rth_speed     = max(0.05, min(1.0, float(g("rth_speed"))))
        self._rth_spacing   = max(0.1, float(g("rth_crumb_spacing")))
        self._rth_min_dist  = float(g("rth_min_dist"))
        self._rth_arrive    = float(g("rth_arrive_tol"))
        self._rth_lead_max  = float(g("rth_lead_max"))
        self._rth_max_terr  = float(g("rth_max_track_err"))
        self._rth_terr_s    = float(g("rth_track_err_s"))
        self._rth_min_clear = float(g("rth_min_clearance"))
        self._rth_block_s   = float(g("rth_block_s"))
        self._rth_yaw_rate  = math.radians(float(g("rth_yaw_rate_dps")))
        self._rth_extra_s   = float(g("rth_timeout_extra_s"))
        self._rth_hover_s   = float(g("rth_hover_s"))
        self._rth_after_goal = bool(g("rth_after_goal"))   # RTHGOAL
        # RTHREPLAN
        mode = str(g("rth_mode")).strip().lower()
        self._rth_mode = mode if mode in ("trail", "replan") else "trail"
        self._rth_replan_to = max(0.0, float(g("rth_replan_timeout_s")))
        self._rth_hard_stopped = False

        self._trail      = []                  # list of np.array([x, y])
        self._trail_lock = threading.Lock()
        self._rth_sp     = None                # (x, y, yaw) while RETURN
        self._rth_ran    = False
        self._rth_result = None                # "home" / stop reason
        self.create_timer(0.2, self._rth_record)

        self.get_logger().info(
            f"{log_tag} Recovery: recoverable planner stops (off-map / stuck / "
            f"timeout) -> RETURN along the flown trail at {self._rth_speed:.2f} m/s "
            f"and land at home. Vehicle problems (geofence / altitude / battery / "
            f"disarm) and pilot actions -> unchanged."
            + ("" if self._rth_enabled else " [rth_enabled:=false — DISABLED]"))
        # RTHREPLAN
        if self._rth_enabled:
            self.get_logger().info(
                f"{log_tag} rth_mode={self._rth_mode}: " + (
                    "the way home is REPLANNED by the FM planner (same loop as "
                    "the outbound leg, obstacle costs and guards included)"
                    if self._rth_mode == "replan" else
                    "the way home retraces the flown trail (no replanning)"))
        # RTHGOAL
        if self._rth_enabled and self._rth_after_goal:
            self.get_logger().info(
                f"{log_tag} rth_after_goal:=true — after the goal is REACHED the "
                f"drone also returns along the trail and lands at home "
                f"(adds roughly path length / {self._rth_speed:.2f} m/s of flight time).")

    # ── breadcrumb trail (5 Hz) ───────────────────────────────────────────
    def _rth_record(self):
        if self._mission_state != self.STATE_FLYING or not self._home_locked:
            return
        p = np.array(self._drone_state.global_pos[:2], dtype=float)
        with self._trail_lock:
            if not self._trail:
                self._trail.append(np.array(self._home_xy, dtype=float))
            if np.linalg.norm(p - self._trail[-1]) >= self._rth_spacing:
                self._trail.append(p)

    def _rth_trail_copy(self):
        with self._trail_lock:
            return [t.copy() for t in self._trail]

    # ── classification ────────────────────────────────────────────────────
    def _rth_eligible(self):
        """(True, category) when RETURN should be attempted, else (False, why)."""
        if not self._rth_enabled:
            return False, "rth_enabled:=false"
        if self._aborted():
            return False, "pilot / link / kill abort"
        if not self._armed:
            return False, "not armed"
        reason = self._abort_reason
        pos = np.array(self._drone_state.global_pos[:2], dtype=float)
        if reason is None:
            d_goal = (float(np.linalg.norm(pos - self._global_target))
                      if self._global_target is not None else float("inf"))
            if self._reached_target or d_goal < 1.0:
                # NEW (2026-09-16, RTHGOAL): opt-in return after a successful
                # mission. Default (False) is the old behaviour: land at the goal.
                if not self._rth_after_goal:
                    return False, "goal reached — landing here is the mission"
                category = "goal reached (rth_after_goal)"
            else:
                category = "mission timeout"
        elif any(reason.startswith(k) for k in self._RTH_LAND_NOW):
            return False, f"vehicle problem ({reason.split(':')[0].split(' ')[0]}) -> land in place"
        elif any(reason.startswith(k) for k in self._RTH_RECOVERABLE):
            category = reason.split(" at ")[0].split(" — ")[0]
        else:
            return False, "unknown stop reason -> land in place"
        d_home = float(np.linalg.norm(pos - self._home_xy))
        if d_home < self._rth_min_dist:
            return False, f"already {d_home:.1f} m from home (< {self._rth_min_dist:.1f})"
        if not self._rth_trail_copy():
            return False, "no trail recorded"
        return True, category

    # ── path helpers ──────────────────────────────────────────────────────
    def _rth_path(self):
        """Current position, then the trail backwards down to home."""
        pos = np.array(self._drone_state.global_pos[:2], dtype=float)
        pts = [pos]
        for c in reversed(self._rth_trail_copy()):
            if np.linalg.norm(c - pts[-1]) > 0.05:
                pts.append(c)
        return pts

    @staticmethod
    def _rth_cumlen(pts):
        cum = [0.0]
        for a, b in zip(pts[:-1], pts[1:]):
            cum.append(cum[-1] + float(np.linalg.norm(b - a)))
        return cum

    @staticmethod
    def _rth_point_at(pts, cum, s):
        if s <= 0.0:
            return pts[0].copy()
        if s >= cum[-1]:
            return pts[-1].copy()
        for i in range(1, len(pts)):
            if s <= cum[i]:
                seg = cum[i] - cum[i - 1]
                w = 0.0 if seg <= 1e-9 else (s - cum[i - 1]) / seg
                return pts[i - 1] + w * (pts[i] - pts[i - 1])
        return pts[-1].copy()

    # ── setpoints while RETURN ────────────────────────────────────────────
    def _publish_cmd(self):
        if self._mission_state == self.STATE_RETURN and self._rth_sp is not None:
            if (self._dry_run or self._rc_override or self._link_lost
                    or self._kill_latched or self._disarm_abort
                    or not self._stream_on):
                return
            x, y, yaw = self._rth_sp
            msg = PositionTarget()
            msg.header.stamp     = self.get_clock().now().to_msg()
            msg.header.frame_id  = "map"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
            msg.position.x = float(x)
            msg.position.y = float(y)
            # nominal cruise z; the barometric variant re-computes it in
            # the wrapped publisher, the plain variant flies it as-is
            msg.position.z = float(self._cruise_z if self._cruise_z is not None
                                   else self._hold_z)
            msg.yaw = float(yaw)
            self._pub_sp.publish(msg)
            return
        super()._publish_cmd()

    # ── the return itself ─────────────────────────────────────────────────
    def _rth_vehicle_problem(self):
        """Checks the base watchdog would do, for the RETURN state."""
        pos = self._drone_state.global_pos
        if self._max_home_d > 0.0:
            d = float(np.linalg.norm(pos[:2] - self._home_xy))
            if d > self._max_home_d:
                return f"GEOFENCE: {d:.1f} m from home"
        if self._min_batt_v > 0.0 and self._batt_v is not None \
                and self._batt_v < self._min_batt_v:
            return f"BATTERY {self._batt_v:.2f} V"
        if self._min_batt_p > 0.0 and self._batt_pct is not None \
                and 0.0 <= self._batt_pct < self._min_batt_p:
            return f"BATTERY {self._batt_pct:.0f}%"
        if not self._armed:
            return "disarmed"
        return None

    # NEW (2026-09-16, RTHREPLAN) ─────────────────────────────────────────
    def _return_home_replan(self, category):
        """Fly home with the FM planner instead of retracing the trail: the
        same replan loop as the outbound leg, with home as the goal. True =
        hovering over home, ready to land there. False = stopped on the way
        (reason in _rth_result); the caller lands in place. Sets
        _rth_hard_stopped when _fly_to_target already stopped the mission."""
        tag = self._rth_tag
        self._rth_ran = True
        pos = np.array(self._drone_state.global_pos[:2], dtype=float)
        d_home = float(np.linalg.norm(pos - self._home_xy))
        timeout = self._rth_replan_to
        if timeout <= 0.0:
            v = max(0.05, float(getattr(self, "_v_max", 0.3)))
            timeout = d_home / v * 2.0 + self._rth_extra_s

        self._announce_phase(
            "RETURN",
            f"{category}: the drone is fine -> flying home with the FM "
            f"planner ({d_home:.1f} m straight line) at "
            f"{float(getattr(self, '_v_max', 0.3)):.2f} m/s, then landing at "
            f"home. Time budget {timeout:.0f} s.", warn=True)
        self._phase_note(
            "the nose turns toward home for this leg. The route is REPLANNED "
            "every replan_period — the drone may take "
            "a different path back. Geofence, altitude, battery, disarm and "
            "the depth guards apply as on the way out. PILOT: RC ready.",
            warn=True)

        # The planner stop that brought us here is handled by flying home, so
        # it must not abort the return leg on its first iteration.
        self._abort_reason = None
        self._reached_target = False
        with self._traj_lock:
            self._traj.invalidate()

        # NEW (2026-09-16, RTHYAW): point the nose at home for the return leg
        # (the FLYING yaw lock otherwise holds the outbound heading, so the
        # drone would fly home backwards and the camera would look away).
        delta = np.asarray(self._home_xy, dtype=float) - pos
        yaw_home = float(math.atan2(delta[1], delta[0])) if d_home > 1e-3 \
            else float(self._drone_state.yaw)
        res = self._fly_to_target(self._home_xy, timeout, tag="RETURN",
                                  banner_extra=" | RETURNING HOME",
                                  timeout_label="return timeout",
                                  yaw_ref=yaw_home)
        if res == "stopped":            # RC / link / kill: already hard-stopped
            self._rth_hard_stopped = True
            self._rth_result = "pilot / link / kill while returning"
            return False
        if res != "reached":
            self._rth_result = self._abort_reason or "return timeout"
            self.get_logger().error("=" * 62)
            self.get_logger().error(
                f"{tag} RETURN STOPPED: {self._rth_result} -> landing in place")
            self.get_logger().error("=" * 62)
            return False

        # Hover over home on the RETURN carrot (no trajectory) before landing.
        with self._traj_lock:
            self._traj.invalidate()
        self._rth_sp = (float(self._home_xy[0]), float(self._home_xy[1]),
                        float(self._drone_state.yaw))
        self._mission_state = self.STATE_RETURN
        self._phase_note(
            f"over home — hovering {self._rth_hover_s:.0f} s, then landing here")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self._rth_hover_s:
            if self._aborted():
                self._rth_result = "pilot / link / kill while returning"
                return False
            problem = self._rth_vehicle_problem()
            if problem is not None:
                self._rth_result = problem
                self.get_logger().error(
                    f"{tag} RETURN STOPPED: {problem} -> landing in place")
                return False
            time.sleep(0.05)
        self._rth_result = "home"
        return True

    def _return_home(self, category):
        """Retrace the trail to home. True = hovering over home, ready to
        land there. False = stopped on the way (reason in _rth_result);
        the caller lands in place. Never raises."""
        tag = self._rth_tag
        pts = self._rth_path()
        cum = self._rth_cumlen(pts)
        total = cum[-1]
        v = self._rth_speed
        timeout = total / v * 2.0 + self._rth_extra_s
        self._rth_ran = True

        self._announce_phase(
            "RETURN",
            f"{category}: the planner stopped, the drone is fine -> returning "
            f"along the flown trail ({len(pts) - 1} legs, {total:.1f} m) at "
            f"{v:.2f} m/s, then landing at home. Time budget {timeout:.0f} s.",
            warn=True)
        self._phase_note(
            f"guards: clearance >= {self._rth_min_clear:.2f} m (known obstacles), "
            f"tracking error < {self._rth_max_terr:.1f} m, geofence, battery. "
            "Any failure -> land in place. PILOT: RC ready.", warn=True)

        with self._traj_lock:
            self._traj.invalidate()
        pos = np.array(self._drone_state.global_pos[:2], dtype=float)
        yaw_cmd = float(self._drone_state.yaw)
        self._rth_sp = (float(pos[0]), float(pos[1]), yaw_cmd)
        self._mission_state = self.STATE_RETURN

        t0 = time.time()
        t_prev = t0
        s = 0.0
        blocked_since = None
        track_since = None
        arrived_at = None
        last_log = 0.0
        stop = None
        try:
            while rclpy.ok():
                now = time.time()
                dt = max(0.0, min(0.2, now - t_prev))
                t_prev = now
                if self._aborted():
                    self._rth_result = "pilot / link / kill"
                    self._rth_sp = None
                    return False
                prob = self._rth_vehicle_problem()
                if prob is not None:
                    stop = prob
                    break
                if now - t0 > timeout:
                    stop = f"time budget {timeout:.0f} s exhausted"
                    break

                pos = np.array(self._drone_state.global_pos[:2], dtype=float)
                carrot = self._rth_point_at(pts, cum, s)
                look = self._rth_point_at(pts, cum, min(total, s + 0.5))
                lead = float(np.linalg.norm(carrot - pos))

                # known-obstacle clearance at the drone and at the carrot
                blocked = False
                c_here = c_car = float("inf")
                if self._esdf.is_ready():
                    c_here = float(self._esdf.get_edt_dis(pos)) + _BODY_RADIUS_M
                    c_car  = float(self._esdf.get_edt_dis(carrot)) + _BODY_RADIUS_M
                    blocked = min(c_here, c_car) < self._rth_min_clear
                if blocked:
                    if blocked_since is None:
                        blocked_since = now
                        self.get_logger().warn(
                            f"{tag} path blocked: clearance here {c_here:.2f} m / "
                            f"ahead {c_car:.2f} m < {self._rth_min_clear:.2f} — holding")
                    elif now - blocked_since > self._rth_block_s:
                        stop = (f"path blocked for {self._rth_block_s:.0f} s "
                                f"(clearance {min(c_here, c_car):.2f} m)")
                        break
                else:
                    blocked_since = None

                # tracking error
                if lead > self._rth_max_terr:
                    if track_since is None:
                        track_since = now
                    elif now - track_since > self._rth_terr_s:
                        stop = f"drone {lead:.1f} m behind the carrot for {self._rth_terr_s:.0f} s"
                        break
                else:
                    track_since = None

                # yaw: nose along the direction of travel (camera looks ahead)
                d_look = look - pos
                yaw_des = yaw_cmd
                if float(np.linalg.norm(d_look)) > 0.15:
                    yaw_des = math.atan2(float(d_look[1]), float(d_look[0]))
                err = _wrap(yaw_des - yaw_cmd)
                step = self._rth_yaw_rate * dt
                yaw_cmd = _wrap(yaw_cmd + max(-step, min(step, err)))
                yaw_ok = abs(_wrap(yaw_des - yaw_cmd)) < math.radians(30.0)

                # advance the carrot only when safe and the drone keeps up
                if not blocked and yaw_ok and lead < self._rth_lead_max:
                    s = min(total, s + v * dt)
                    carrot = self._rth_point_at(pts, cum, s)
                self._rth_sp = (float(carrot[0]), float(carrot[1]), yaw_cmd)

                # arrival
                d_home = float(np.linalg.norm(pos - pts[-1]))
                if s >= total - 1e-6 and d_home < self._rth_arrive:
                    if arrived_at is None:
                        arrived_at = now
                        self._phase_note(
                            f"over home ({d_home:.2f} m) — hovering "
                            f"{self._rth_hover_s:.0f} s, then landing here")
                    elif now - arrived_at >= self._rth_hover_s:
                        self._rth_result = "home"
                        return True
                else:
                    arrived_at = None

                if now - last_log >= 2.0:
                    last_log = now
                    self.get_logger().info(
                        f"{tag} {s:5.1f}/{total:.1f} m | home {d_home:4.1f} m | "
                        f"lead {lead:4.2f} m | clear {min(c_here, c_car):5.2f} m"
                        f"{' BLOCKED' if blocked else ''}{'' if yaw_ok else ' turning'}")
                time.sleep(0.05)
        except Exception as e:      # pragma: no cover - never leave the drone without a verdict
            stop = f"internal error ({e!r})"
        if stop is None:
            stop = "ROS shut down"
        self._rth_result = stop
        self.get_logger().error("=" * 62)
        self.get_logger().error(f"{tag} RETURN STOPPED: {stop} -> landing in place")
        self.get_logger().error("=" * 62)
        # keep hovering at the current position until the caller lands
        pos = np.array(self._drone_state.global_pos[:2], dtype=float)
        self._rth_sp = (float(pos[0]), float(pos[1]), yaw_cmd)
        return False

    # ── hook: try RETURN before the inherited landing ─────────────────────
    def _land_and_finish(self):
        if not self._rth_ran and not self._aborted():
            ok, why = self._rth_eligible()
            if ok:
                # RTHREPLAN: "replan" flies home with the planner instead.
                if self._rth_mode == "replan":
                    came_home = self._return_home_replan(why)
                    if self._rth_hard_stopped:
                        return None     # _fly_to_target already stopped it
                else:
                    came_home = self._return_home(why)
                if self._aborted():
                    return self._hard_stop("RC override / link loss / kill while returning")
                self._phase_note(
                    "landing at home" if came_home else
                    f"landing here ({self._rth_result})", warn=not came_home)
            else:
                self.get_logger().info(f"{self._rth_tag} no return: {why}")
        return super()._land_and_finish()
