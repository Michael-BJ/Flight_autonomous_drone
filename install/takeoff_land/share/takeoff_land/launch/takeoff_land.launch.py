#!/usr/bin/env python3
"""
takeoff_land.launch.py
======================
All-in-one: MAVROS (serial) + px4_sensor_reader + takeoff_land_node.
For testing TAKE OFF -> HOVER -> LANDING via OFFBOARD on real hardware.

USAGE:
    ros2 launch takeoff_land takeoff_land.launch.py fcu_url:=/dev/ttyTHS1:921600
    ros2 launch takeoff_land takeoff_land.launch.py \
        fcu_url:=/dev/ttyACM0:57600 target_alt:=1.5 hover_time:=10.0

PARAMS: fcu_url, target_alt, hover_time, cmd_hz, descent_speed, auto_land_mode,
        land_handoff_alt, rc_override_enabled, verify_rc_override_param,
        max_pos_error, max_vz, max_alt_error

START ORDER (with delays):
    t=0   : kill stale processes + start MAVROS
    t=10s : start px4_sensor_reader
    t=14s : start takeoff_land_node (begins handshake after telemetry flows)
"""
import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_MAVROS_CFG = "/opt/ros/humble/share/mavros/launch/px4_config.yaml"
_LOCAL_PLG  = "/opt/ros/humble/share/mavros/launch/px4_pluginlists.yaml"


def _kill_stale(context):
    import time, glob
    try:
        subprocess.run(["pkill", "-9", "-f", "mavros_node"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "px4_sensor_reader"], capture_output=True)
        subprocess.run(["pkill", "-9", "-f", "takeoff_land_node"], capture_output=True)
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
    arg_fcu_url   = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyACM0:57600",
        description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600")
    arg_alt       = DeclareLaunchArgument("target_alt",     default_value="1.2")
    arg_hover     = DeclareLaunchArgument("hover_time",     default_value="8.0")
    arg_cmd_hz    = DeclareLaunchArgument("cmd_hz",         default_value="50")
    arg_descent   = DeclareLaunchArgument("descent_speed",  default_value="0.3")
    arg_auto_land = DeclareLaunchArgument("auto_land_mode", default_value="true")
    arg_handoff   = DeclareLaunchArgument("land_handoff_alt",    default_value="0.25")
    arg_rc_ovr    = DeclareLaunchArgument("rc_override_enabled", default_value="true")
    arg_verify_rc = DeclareLaunchArgument(
        "verify_rc_override_param", default_value="true",
        description="Pre-flight check of PX4 COM_RC_OVERRIDE. Set false only "
                     "for bench tests without a battery/RC bound.")
    arg_max_pos   = DeclareLaunchArgument("max_pos_error",  default_value="2.0")
    arg_max_vz    = DeclareLaunchArgument("max_vz",         default_value="3.0")
    arg_max_alt   = DeclareLaunchArgument("max_alt_error",  default_value="1.5")

    fcu_url       = LaunchConfiguration("fcu_url")
    target_alt    = LaunchConfiguration("target_alt")
    hover_time    = LaunchConfiguration("hover_time")
    cmd_hz        = LaunchConfiguration("cmd_hz")
    descent_speed = LaunchConfiguration("descent_speed")
    auto_land     = LaunchConfiguration("auto_land_mode")
    land_handoff  = LaunchConfiguration("land_handoff_alt")
    rc_override   = LaunchConfiguration("rc_override_enabled")
    verify_rc     = LaunchConfiguration("verify_rc_override_param")
    max_pos_error = LaunchConfiguration("max_pos_error")
    max_vz        = LaunchConfiguration("max_vz")
    max_alt_error = LaunchConfiguration("max_alt_error")

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

    takeoff_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="takeoff_land",
                executable="takeoff_land_node",
                name="takeoff_land_node",
                output="screen",
                parameters=[{
                    "target_alt":          ParameterValue(target_alt,    value_type=float),
                    "hover_time":          ParameterValue(hover_time,    value_type=float),
                    "cmd_hz":              ParameterValue(cmd_hz,        value_type=int),
                    "descent_speed":       ParameterValue(descent_speed, value_type=float),
                    "auto_land_mode":      ParameterValue(auto_land,     value_type=bool),
                    "land_handoff_alt":    ParameterValue(land_handoff,  value_type=float),
                    "rc_override_enabled": ParameterValue(rc_override,   value_type=bool),
                    "verify_rc_override_param": ParameterValue(verify_rc, value_type=bool),
                    "max_pos_error":       ParameterValue(max_pos_error, value_type=float),
                    "max_vz":              ParameterValue(max_vz,        value_type=float),
                    "max_alt_error":       ParameterValue(max_alt_error, value_type=float),
                    "use_ekf_stable":      True,
                }],
            ),
        ],
    )

    return LaunchDescription([
        arg_fcu_url, arg_alt, arg_hover, arg_cmd_hz, arg_descent, arg_auto_land,
        arg_handoff, arg_rc_ovr, arg_verify_rc, arg_max_pos, arg_max_vz, arg_max_alt,
        kill_old, mavros_node, reader, takeoff_node,
    ])
