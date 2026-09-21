#!/usr/bin/env python3
"""
fm_inference_baro_node.py — fm_deploy with BAROMETRIC altitude hold + RETURN recovery
==========================================================================
    FMInferenceBaroNode = BaroHoldMixin + ReturnHomeRecoveryMixin
                          + FMInferenceRealNode

Everything the planner and the hardware safety layer do is inherited from
fm_deploy (fm_inference_base / fm_inference_node / fm_inference_real_node
are NOT modified). This node changes only:

  ALTITUDE (see takeoff_land_barometer/baro_altitude.py)
    - ground reference from barometer + EKF instead of EKF only
    - every position setpoint's z is re-computed as
          z_ekf + gain * (alt_target - alt_baro)
      so PX4 holds the BAROMETRIC altitude while x/y stay GPS/EKF
    - takeoff "reached altitude" and the in-flight altitude watchdog use
      the barometric altitude (the inherited watchdog compared the EKF z
      with cruise_z, which is exactly the quantity that is now EXPECTED
      to drift — it is disabled and replaced)
    - a lost barometer stream is an abort (land in place)
    - the octomap band is widened UPWARD by band_drift_margin: obstacle
      points are inserted at the (drifting) EKF z, so the band tolerates
      that drift. Not widened downward: with EKF drift the ground itself
      would enter the band. Recommendation: target_alt >= 2.0 m.

  RECOVERY (see recovery.py)
    - a recoverable planner stop (off-map / stuck / mission timeout) returns
      along the flown trail to home and lands there instead of landing in
      the middle of the course; vehicle problems and pilot actions behave
      exactly as before.

PX4 parameters are NOT touched (write_px4_params stays false).
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter as RclParameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue as RclParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Empty as EmptySrv

from fm_deploy.fm_inference_real_node import FMInferenceRealNode, _PHASE_COLOR
from takeoff_land_barometer.baro_altitude import BaroHoldMixin
from fm_deploy_barometer.recovery import ReturnHomeRecoveryMixin


class FMInferenceBaroNode(BaroHoldMixin, ReturnHomeRecoveryMixin, FMInferenceRealNode):

    def __init__(self):
        super().__init__()
        self.declare_parameter("band_drift_margin", 0.5)   # m, octomap band upward
        self._band_margin = max(0.0, float(self.get_parameter("band_drift_margin").value))
        # The inherited watchdog's altitude test is on the EKF z; with the
        # barometric loop the EKF z is expected to wander away from cruise_z.
        # Disable it there (0 = off) and test the barometric altitude instead.
        self._baro_max_alt_err = float(self._max_alt_err)
        self._max_alt_err = 0.0
        self._rth_init("[RTH]")
        self._baro_init("[BARO]")
        self.get_logger().info(
            f"[BARO] altitude watchdog: |baro alt - {self._alt:.2f}| <= "
            f"{self._baro_max_alt_err:.2f} m (inherited EKF-z test disabled)")

    # ── takeoff: reached altitude? (barometric) ───────────────────────────
    def _wait_altitude_real(self, target_z, tol=0.15, timeout=30.0):
        alt_t = float(target_z) - self._ground_z
        t0, stable = time.time(), None
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._aborted() or self._abort_reason is not None:
                return False
            alt = self._baro_alt()
            vz  = abs(self._baro_ekf_vz())
            if abs(alt - alt_t) < tol and vz < 0.2:
                stable = stable or time.time()
                if time.time() - stable >= 2.0:
                    return True
            else:
                stable = None
            time.sleep(0.1)
        return False

    # ── watchdog: inherited checks + barometric altitude + baro health ────
    def _watchdog(self):
        super()._watchdog()
        if self._mission_state not in (self.STATE_TAKEOFF, self.STATE_FLYING,
                                       self.STATE_RETURN):
            return
        if self._abort_reason is not None:
            return
        if (self._cruise_z is not None and self._baro_max_alt_err > 0.0
                and self._mission_state in (self.STATE_FLYING, self.STATE_RETURN)):
            alt = self._baro_alt()
            alt_t = float(self._cruise_z) - self._ground_z
            dz = abs(alt - alt_t)
            if dz > self._baro_max_alt_err:
                self._abort_reason = (
                    f"ALTITUDE deviated {dz:.2f} m from cruise (barometric "
                    f"{alt:.2f} vs {alt_t:.2f} m, limit {self._baro_max_alt_err:.2f} m)")
                return
        prob = self._baro_health_problem()
        if prob is not None:
            self._abort_reason = prob

    # ── octomap band: same as the real node, widened upward ───────────────
    def _configure_octomap_band(self, alt_above_ground):
        occ_min = self._ground_z + max(0.35, alt_above_ground - 0.7)
        occ_max = self._ground_z + alt_above_ground + 1.0 + self._band_margin
        if not self._octo_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"[BARO] octomap set_parameters absent — set manually: "
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
            f"[BARO] {'ok' if ok else 'FAILED'} Octomap band "
            f"[{occ_min:.2f}, {occ_max:.2f}] odom z = "
            f"[{occ_min - self._ground_z:.2f}, {occ_max - self._ground_z:.2f}] m "
            f"above ground (ground_z={self._ground_z:.2f}, +{self._band_margin:.1f} m "
            f"drift margin on top)")
        if ok and self._octo_reset_client.wait_for_service(timeout_sec=3.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(),
                           timeout=5.0)

    # ── status line: barometric altitude first ───────────────────────────
    def _print_status(self):
        pos   = self._drone_state.global_pos
        speed = float(np.linalg.norm(self._drone_state.global_vel[:2]))
        col   = self._c(_PHASE_COLOR.get(self._phase_name, "white"))
        rst   = self._c("reset")
        bold  = self._c("bold")

        alt, bmode = self._baro.altitude()
        parts = [f"balt={alt:5.2f}m({bmode})",
                 f"ekf_alt={float(pos[2]) - self._ground_z:5.2f}m"]
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
        if self._kill_now:
            parts.append(f"{self._c('red')}KILL{rst}")
        st = self._baro.stale_for()
        parts.append(f"baro={self._baro.rate_hz():.0f}Hz"
                     + (f" {self._c('red')}STALE {st:.1f}s{rst}"
                        if st > self._baro.stale_s else ""))
        act = ""
        if self._mission_state == self.STATE_FLYING:
            if self._escape_active:
                act = f" {self._c('red')}{bold}[ESCAPE]{rst}"
            else:
                with self._traj_lock:
                    moving = self._traj.is_valid()
                act = ("" if moving
                       else f" {self._c('yellow')}[HOLDING - no trajectory]{rst}")
        elif self._mission_state == self.STATE_RETURN:
            act = f" {self._c('yellow')}[RETURN to home]{rst}"
        inf = np.mean(self._inference_times) if self._inference_times else 0.0
        tail = (f"replan#{self._replan_count} {self._last_replan_ms:.0f}ms "
                f"(inf {inf:.0f}ms)")
        if self._replan_slow_n:
            tail += f" {self._c('yellow')}slow x{self._replan_slow_n}{rst}"
        self.get_logger().info(
            f"{col}{bold}[{self._elapsed()}] {self._phase_name:<9s}{rst}"
            f"{act} | " + " | ".join(parts) + f" | {self._c('dim')}{tail}{rst}")


def main(args=None):
    rclpy.init(args=args)
    node = FMInferenceBaroNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    seq = threading.Thread(target=node.run_sequence, daemon=True)
    seq.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("[BARO] Ctrl-C — setpoints stopped.")
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
