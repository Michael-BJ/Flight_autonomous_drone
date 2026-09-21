#!/usr/bin/env python3
"""
vfh_perception.launch.py
========================
PERCEPTION ONLY — no MAVROS, no flight node, nothing arms. Use it on the
bench to look at what VFH sees:

    Orbbec Gemini 2 driver (gemini2.launch.py, depth only)
        /camera/depth/image_raw (16UC1 mm)
    -> fm_deploy gemini2_depth_bridge_node
        /realsense/depth/float32 (32FC1 m, 640x480), /realsense/depth/camera_info
    -> vfh_avoidance_node
        /vfh/movement_direction, /vfh/obstacles, /vfh/visual

    ros2 launch vfh_avoidance_barometer vfh_perception.launch.py
    ros2 topic echo /vfh/movement_direction
    ros2 run rqt_image_view rqt_image_view /vfh/visual      (with a display)

use_camera:=false skips the Orbbec driver + bridge (feed
/realsense/depth/float32 yourself, e.g. from a bag).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from vfh_avoidance_barometer.launch_common import (
    camera_arguments, camera_include, depth_bridge_node,
    vfh_arguments, vfh_node)


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="false = do not start the Orbbec driver + bridge"),
    ] + camera_arguments() + vfh_arguments()

    camera = TimerAction(period=1.0, actions=[camera_include()],
                         condition=IfCondition(LaunchConfiguration("use_camera")))
    bridge = TimerAction(period=4.0, actions=[depth_bridge_node()],
                         condition=IfCondition(LaunchConfiguration("use_camera")))
    vfh = TimerAction(period=5.0, actions=[vfh_node()])
    return LaunchDescription(args + [camera, bridge, vfh])
