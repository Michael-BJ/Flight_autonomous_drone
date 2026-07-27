#!/usr/bin/env python3
"""
mavros_only.launch.py
=====================
Bring up the communication layer: MAVROS (serial) + px4_sensor_reader.
Run this FIRST, then start the mission in a second terminal:

    Terminal 1:
        ros2 launch takeoff_land mavros_only.launch.py fcu_url:=/dev/ttyTHS1:921600
    Terminal 2:
        ros2 run takeoff_land takeoff_land_node --ros-args -p target_alt:=1.2

NOTE: do NOT set name="mavros" on the MAVROS Node (causes double prefix
/mavros/mavros/ -> crash). Let MAVROS use its default name.
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_MAVROS_CFG = "/opt/ros/humble/share/mavros/launch/px4_config.yaml"
_LOCAL_PLG  = "/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml"


def _kill_stale(context):
    """Kill stale MAVROS/reader processes and clean FastRTPS shared memory."""
    import time, glob
    try:
        subprocess.run(["pkill", "-9", "-f", "mavros_node"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "px4_sensor_reader"], capture_output=True)
    except Exception:
        pass
    time.sleep(2.0)
    for shm in glob.glob("/dev/shm/fastrtps_*"):
        try:
            os.remove(shm)
        except Exception:
            pass
    subprocess.run(["ros2", "daemon", "stop"], capture_output=True)
    time.sleep(1.0)
    subprocess.run(["ros2", "daemon", "start"], capture_output=True)
    time.sleep(1.0)
    return []


def generate_launch_description():
    arg_fcu_url = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:57600",
        description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600",
    )
    fcu_url = LaunchConfiguration("fcu_url")

    kill_old = OpaqueFunction(function=_kill_stale)

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        output="screen",
        parameters=[
            {"fcu_url":             fcu_url},
            {"gcs_url":             ""},
            {"target_system_id":    1},
            {"target_component_id": 1},
            {"fcu_protocol":        "v2.0"},
            _MAVROS_CFG,
            _LOCAL_PLG,
        ],
    )

    # delay 10s so MAVROS has time to connect to PX4 before reader subscribes
    reader = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="takeoff_land",
                executable="px4_sensor_reader",
                name="px4_sensor_reader",
                output="screen",
                parameters=[{"fcu_url": fcu_url, "auto_launch_mavros": False}],
            ),
        ],
    )

    return LaunchDescription([arg_fcu_url, kill_old, mavros_node, reader])
