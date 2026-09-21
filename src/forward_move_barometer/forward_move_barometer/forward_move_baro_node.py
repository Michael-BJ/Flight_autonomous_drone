#!/usr/bin/env python3
"""
forward_move_baro_node.py
==================================================================
PURPOSE: TAKE OFF -> HOVER -> FORWARD -> HOLD -> LANDING at the end point,
exactly like forward_move, but the ALTITUDE is measured with the BAROMETER
instead of the EKF z (which follows the drifting GPS height on this
drone). X and Y stay GPS/EKF.

NOT A COPY:
    ForwardMoveBaroNode = TakeoffLandBaroMixin + ForwardMoveNode.
    forward_move's run_sequence (the FORWARD leg, the moving sanity
    anchor, the abort dispatch) and everything it inherits from
    takeoff_land are used as-is. The barometric overrides are the same
    ones takeoff_land_barometer uses (see takeoff_land_baro_node.py and
    baro_altitude.py): ground reference, _wait_altitude, _sanity_check
    (so the tracking-error limit during FORWARD is unchanged and the
    altitude limit is barometric), the status line, and the wrapped
    setpoint publisher.

NO OBSTACLE AVOIDANCE (same as forward_move). PX4 parameters are NOT
touched.
"""
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from forward_move.forward_move_node import ForwardMoveNode
from takeoff_land_barometer.takeoff_land_baro_node import TakeoffLandBaroMixin


class ForwardMoveBaroNode(TakeoffLandBaroMixin, ForwardMoveNode):

    def __init__(self):
        super().__init__()          # ForwardMoveNode -> TakeoffLandNode
        self._baro_init("[BARO]")


def main(args=None):
    argv = list(sys.argv if args is None else args)
    if not any(a.startswith("__node:=") or a.startswith("__name:=") for a in argv):
        argv += ["--ros-args", "-r", "__node:=forward_move_baro_node"]
    rclpy.init(args=argv)
    node = ForwardMoveBaroNode()

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
