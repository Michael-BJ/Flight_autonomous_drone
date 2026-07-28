#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

_MAVROS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)


class MavrosTfBroadcasterNode(Node):

    def __init__(self):
        super().__init__("mavros_tf_broadcaster_node")

        self.declare_parameter("pose_topic",   "/mavros/local_position/pose")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame",  "base_link")

        pose_topic  = self.get_parameter("pose_topic").value
        self.parent = self.get_parameter("parent_frame").value
        self.child  = self.get_parameter("child_frame").value

        self.tf_br = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            PoseStamped, pose_topic, self._on_pose, _MAVROS_QOS)

        self.get_logger().info(
            f"MavrosTfBroadcasterNode active | "
            f"{pose_topic} -> TF {self.parent}->{self.child} | QoS: BEST_EFFORT")

    def _on_pose(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self.parent
        t.child_frame_id  = self.child
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation      = msg.pose.orientation
        self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MavrosTfBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
