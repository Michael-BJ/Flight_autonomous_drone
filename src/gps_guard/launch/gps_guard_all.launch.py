#!/usr/bin/env python3
"""One-command bring-up: MAVROS + px4_sensor_reader + gps_check_node.

Convenience wrapper so a GPS check needs ONE terminal instead of two. It
includes takeoff_land's mavros_only.launch.py (which starts MAVROS and the
px4_sensor_reader) and then starts gps_check_node once telemetry is
flowing.

USAGE:
    ros2 launch gps_guard gps_guard_all.launch.py fcu_url:=/dev/ttyACM0:57600
    ros2 launch gps_guard gps_guard_all.launch.py \\
        fcu_url:=/dev/ttyTHS1:921600 min_satellites:=10 max_hdop:=1.5

REQUIREMENTS / CAVEATS:
    - The takeoff_land package must be built and sourced (it provides both
      px4_sensor_reader and the mavros_only launch file wrapped here).
    - Do NOT run this together with takeoff_land.launch.py — both bring up
      their own MAVROS + reader, which would create duplicate subscribers.
    - This only CHECKS GPS. It does not arm or fly anything.

START ORDER (inherited from mavros_only.launch.py + this file):
    t=0    : kill stale processes + start MAVROS
    t=10s  : start px4_sensor_reader
    t=13s  : start gps_check_node
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg_fcu = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:57600",
        description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600")

    gps_args = [
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

    fcu_url = LaunchConfiguration("fcu_url")

    def cfg(name):
        return LaunchConfiguration(name)

    # Reuse takeoff_land's comm layer instead of duplicating MAVROS + the
    # kill-stale housekeeping. mavros_only.launch.py starts MAVROS at t=0
    # and px4_sensor_reader at t=10s.
    comm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("takeoff_land"),
            "launch", "mavros_only.launch.py")),
        launch_arguments={"fcu_url": fcu_url}.items(),
    )

    # Start the checker after the reader is up (reader starts at t=10s).
    gps_check = TimerAction(
        period=13.0,
        actions=[
            Node(
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
            ),
        ],
    )

    return LaunchDescription([arg_fcu] + gps_args + [comm, gps_check])
