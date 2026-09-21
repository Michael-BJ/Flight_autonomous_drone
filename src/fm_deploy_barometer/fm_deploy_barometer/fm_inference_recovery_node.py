#!/usr/bin/env python3
"""
fm_inference_recovery_node.py — fm_deploy (EKF/GPS altitude, unchanged) + RETURN recovery
==========================================================================
    FMInferenceRecoveryNode = ReturnHomeRecoveryMixin + FMInferenceRealNode

Altitude is the inherited EKF z (GPS height reference), exactly like
fm_deploy. The only addition is the recovery ladder of recovery.py: a
recoverable planner stop (off-map / stuck / mission timeout) returns along
the flown trail and lands at home instead of landing in place.

Use this to test the recovery behaviour on its own, or on a day when the
GPS height is fine. Select it with use_baro:=false in
fm_all_barometer.launch.py / fm_real_barometer.launch.py.
"""
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from fm_deploy.fm_inference_real_node import FMInferenceRealNode
from fm_deploy_barometer.recovery import ReturnHomeRecoveryMixin


class FMInferenceRecoveryNode(ReturnHomeRecoveryMixin, FMInferenceRealNode):

    def __init__(self):
        super().__init__()
        self._rth_init("[RTH]")


def main(args=None):
    rclpy.init(args=args)
    node = FMInferenceRecoveryNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    seq = threading.Thread(target=node.run_sequence, daemon=True)
    seq.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("[RTH] Ctrl-C — setpoints stopped.")
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
