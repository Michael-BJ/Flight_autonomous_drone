#!/usr/bin/env python3
"""
takeoff_land_baro_node.py
==================================================================
PURPOSE: TAKE OFF -> HOVER -> LANDING via OFFBOARD, exactly like
takeoff_land, but the ALTITUDE is measured with the BAROMETER instead of
the EKF z (which follows the drifting GPS height on this drone). X and Y
stay GPS/EKF.

THIS IS takeoff_land's OWN CODE, NOT A COPY:
    TakeoffLandBaroNode = BaroHoldMixin + TakeoffLandNode. The mission
    sequence (run_sequence), every pre-flight gate (COM_RC_OVERRIDE, GPS,
    RC mode switch, kill switch), the ARM/OFFBOARD handshake, the controlled
    descent, the AUTO.LAND hand-off and every abort path are the inherited
    methods. Only these are replaced (see baro_altitude.py for the why):
      _wait_ekf_stable   -> ground reference from barometer + EKF
      _wait_altitude     -> barometric altitude instead of EKF z
      _sanity_check      -> altitude limit on the barometric altitude,
                            plus "barometer stream lost" abort
      _print_status      -> shows the barometric altitude
      the setpoint publisher is wrapped so every position setpoint's z is
      re-computed as z_ekf + gain * (alt_target - alt_baro)

PX4 parameters are NOT touched. See baro_altitude.py for the estimator,
its limits (ground effect, warm-up drift) and the pre-flight criteria.

FSM FLOW (unchanged):
    IDLE -> (connect) -> (GPS gate) -> (ground reference) -> READY
         -> ARM -> OFFBOARD -> TAKEOFF -> HOVER
         -> LANDING (controlled descent -> AUTO.LAND -> auto-disarm) -> DONE
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor

from takeoff_land.takeoff_land_node import TakeoffLandNode, _PHASE_COLOR
from takeoff_land_barometer.baro_altitude import BaroHoldMixin


class TakeoffLandBaroMixin(BaroHoldMixin):
    """The takeoff_land-family overrides (shared with forward_move_barometer)."""

    # ── Wait for altitude (barometric) ───────────────────────────────────
    def _wait_altitude(self, target_z, tol=0.15, vz_tol=0.2,
                       stable_dur=2.0, timeout=30.0) -> bool:
        """Same contract as takeoff_land's: True when the altitude is stable
        at the target, False on timeout / abort. Altitude = barometric."""
        alt_target = float(target_z) - self._ground_z
        t0 = time.time()
        t_stable = None
        while time.time() - t0 < timeout and rclpy.ok():
            if self._rc_override or self._link_lost:
                return False
            if self._kill_latched or self._disarm_abort:
                return False
            if not self._sanity_check(target_z, check_alt=False, check_vz=False):
                return False
            alt = self._baro_alt()
            vz  = abs(self._baro_ekf_vz())
            if abs(alt - alt_target) < tol and vz < vz_tol:
                if t_stable is None:
                    t_stable = time.time()
                elif time.time() - t_stable >= stable_dur:
                    return True
            else:
                t_stable = None
            time.sleep(0.1)
        return False

    # ── Sanity check (barometric altitude) ───────────────────────────────
    def _sanity_check(self, target_z: float, check_alt: bool = True,
                      check_vz: bool = True) -> bool:
        """Horizontal drift and vz limits are the inherited checks; the
        altitude limit is measured with the barometric altitude, and a lost
        barometer stream is an abort of its own (the loop would otherwise be
        flying on EKF dead-reckoning indefinitely)."""
        if not super()._sanity_check(target_z, check_alt=False, check_vz=check_vz):
            return False
        problem = self._baro_health_problem()
        if problem is not None:
            self._sanity_abort(problem)
            return False
        if check_alt:
            alt = self._baro_alt()
            alt_t = float(target_z) - self._ground_z
            if abs(alt - alt_t) > self._max_alt_error:
                self._sanity_abort(
                    f"Altitude deviation: baro alt={alt:.2f}m target={alt_t:.2f}m "
                    f"(limit ±{self._max_alt_error:.1f}m)")
                return False
        return True

    # ── Status line: barometric altitude first ───────────────────────────
    def _print_status(self):
        col  = self._c(_PHASE_COLOR.get(self._phase_name, "white"))
        rst  = self._c("reset")
        bold = self._c("bold")
        if not self._connected:
            self.get_logger().warn(
                f"[{self._elapsed()}] {self._phase_name} — waiting for "
                "/px4/state (px4_sensor_reader & MAVROS)...")
            return
        alt, mode = self._baro.altitude()
        ekf_alt = float(self._pos[2]) - self._ground_z
        parts = [f"balt={alt:5.2f}m({mode})", f"ekf_alt={ekf_alt:5.2f}m",
                 f"z={float(self._pos[2]):6.2f}m", f"vz={float(self._vel[2]):+5.2f}"]
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
        sent = self._pub_sp.last_sent
        parts.append(f"sp_z={float(self._sp_z):5.2f}"
                     + (f"->{sent:5.2f}" if sent is not None else ""))
        arm_c = self._c("red") if self._armed else self._c("green")
        parts.append(f"{arm_c}{'ARMED' if self._armed else 'disarmed'}{rst}")
        parts.append(f"mode={self._mode or '?'}")
        if self._kill_now:
            parts.append(f"{self._c('red')}KILL{rst}")
        if self._gps_q:
            sats = self._gps_q.get("satellites")
            hdop = self._gps_q.get("hdop")
            parts.append(f"gps={'?' if sats is None else sats}sat/"
                         f"{'?' if hdop is None else f'{float(hdop):.1f}'}")
        st = self._baro.stale_for()
        parts.append(f"baro={self._baro.rate_hz():.0f}Hz"
                     + (f" {self._c('red')}STALE {st:.1f}s{rst}"
                        if st > self._baro.stale_s else ""))
        self.get_logger().info(
            f"{col}{bold}[{self._elapsed()}] {self._phase_name:<9s}{rst} | "
            + " | ".join(parts))


class TakeoffLandBaroNode(TakeoffLandBaroMixin, TakeoffLandNode):

    def __init__(self):
        super().__init__()          # TakeoffLandNode: params, pubs, timers
        self._baro_init("[BARO]")   # estimator + publisher wrap + params


def main(args=None):
    # TakeoffLandNode names itself "takeoff_land_node". Rename this process's
    # node unless the caller already remapped it (the launch file does).
    argv = list(sys.argv if args is None else args)
    if not any(a.startswith("__node:=") or a.startswith("__name:=") for a in argv):
        argv += ["--ros-args", "-r", "__node:=takeoff_land_baro_node"]
    rclpy.init(args=argv)
    node = TakeoffLandBaroNode()

    th = threading.Thread(target=node.run_sequence, daemon=True)
    th.start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("[BARO] Stopped by user (Ctrl-C).")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
