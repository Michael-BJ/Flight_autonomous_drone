#!/usr/bin/env python3
"""Launch the standalone GPS pre-flight / stability checker.

This launches ONLY gps_check_node. It needs a px4_sensor_reader
(from takeoff_land or fm_deploy) already publishing /px4/state and
/px4/sensors — it deliberately does not start its own MAVROS bridge, so
there is never a second MAVROS subscriber in the system.

USAGE:
    ros2 launch gps_guard gps_guard.launch.py
    ros2 launch gps_guard gps_guard.launch.py min_satellites:=10 max_hdop:=1.5
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("min_fix_type",    default_value="3"),
        DeclareLaunchArgument("min_satellites",  default_value="8"),
        DeclareLaunchArgument("max_hdop",        default_value="2.0"),
        DeclareLaunchArgument("require_h_acc",   default_value="false"),
        DeclareLaunchArgument("max_h_acc",       default_value="5.0"),
        DeclareLaunchArgument("stable_dur",      default_value="5.0"),
        DeclareLaunchArgument("window_s",        default_value="5.0"),
        DeclareLaunchArgument("max_sat_swing",   default_value="4"),
        DeclareLaunchArgument("max_hdop_swing",  default_value="0.8"),
        DeclareLaunchArgument("max_pos_drift",   default_value="0.30"),
    ]

    def cfg(name):
        return LaunchConfiguration(name)

    node = Node(
        package="gps_guard",
        executable="gps_check_node",
        name="gps_check_node",
        output="screen",
        parameters=[{
            "min_fix_type":   ParameterValue(cfg("min_fix_type"),   value_type=int),
            "min_satellites": ParameterValue(cfg("min_satellites"), value_type=int),
            "max_hdop":       ParameterValue(cfg("max_hdop"),       value_type=float),
            "require_h_acc":  ParameterValue(cfg("require_h_acc"),  value_type=bool),
            "max_h_acc":      ParameterValue(cfg("max_h_acc"),      value_type=float),
            "stable_dur":     ParameterValue(cfg("stable_dur"),     value_type=float),
            "window_s":       ParameterValue(cfg("window_s"),       value_type=float),
            "max_sat_swing":  ParameterValue(cfg("max_sat_swing"),  value_type=int),
            "max_hdop_swing": ParameterValue(cfg("max_hdop_swing"), value_type=float),
            "max_pos_drift":  ParameterValue(cfg("max_pos_drift"),  value_type=float),
        }],
    )

    return LaunchDescription(args + [node])
