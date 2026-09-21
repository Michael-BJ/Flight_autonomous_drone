#!/usr/bin/env python3
"""
forward_move_barometer.launch.py
================================
All-in-one: MAVROS (serial) + px4_sensor_reader (from takeoff_land)
+ forward_move_baro_node.
TAKE OFF -> HOVER -> FORWARD -> HOLD -> LANDING at the end point, altitude
from the BAROMETER (x/y from GPS/EKF as before). PX4 parameters are NOT
touched. NO obstacle avoidance.

Arguments = takeoff_land_barometer.launch.py's (same names, same BARO
defaults: max_ground_z 1000, max_ground_drift 0.5, ekf_gate_std 0.15,
baro_*) + forward_move's forward_distance / forward_speed /
forward_hold_time.

USAGE:
    ros2 launch forward_move_barometer forward_move_barometer.launch.py \
        forward_distance:=1.0 forward_speed:=0.3 target_alt:=2.0

START ORDER (with delays):
    t=0   : kill stale processes + start MAVROS
    t=10s : start px4_sensor_reader
    t=14s : start forward_move_baro_node
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node

# The argument/parameter tables of takeoff_land_barometer.launch.py, so both
# launch files stay identical where they overlap.
from takeoff_land_barometer.launch_common import (
    takeoff_land_arguments, takeoff_land_parameters, mavros_and_reader, _f)


def generate_launch_description():
    args = takeoff_land_arguments() + [
        DeclareLaunchArgument(
            "forward_distance", default_value="2.0",
            description="Metres to fly straight ahead along the heading the nose "
                        "points at when the mission starts (max 10). NO obstacle "
                        "avoidance — the path must be clear."),
        DeclareLaunchArgument(
            "forward_speed", default_value="0.3",
            description="Forward setpoint speed in m/s (0.05-1.0)."),
        DeclareLaunchArgument(
            "forward_hold_time", default_value="3.0",
            description="Seconds to hold at the end point before landing there."),
    ]
    params = takeoff_land_parameters()
    params.update({
        "forward_distance":  _f("forward_distance"),
        "forward_speed":     _f("forward_speed"),
        "forward_hold_time": _f("forward_hold_time"),
    })
    flight_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="forward_move_barometer",
                executable="forward_move_baro_node",
                name="forward_move_baro_node",
                output="screen",
                parameters=[params],
            ),
        ],
    )
    return LaunchDescription(args + mavros_and_reader() + [flight_node])
