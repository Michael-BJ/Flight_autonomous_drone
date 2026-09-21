#!/usr/bin/env python3
"""
takeoff_land_barometer.launch.py
================================
All-in-one: MAVROS (serial) + px4_sensor_reader (from takeoff_land)
+ takeoff_land_baro_node.
TAKE OFF -> HOVER -> LANDING via OFFBOARD, altitude from the BAROMETER
(x/y from GPS/EKF as before). PX4 parameters are NOT touched.

Every takeoff_land argument below has the SAME name as in
takeoff_land.launch.py. Defaults that differ (tagged BARO):
    max_ground_z      1000.0  (was 1.0)  the absolute EKF z is irrelevant to
                                         the barometric loop (2026-09-13: the
                                         EKF read -2.7..-7.4 m on the ground)
    max_ground_drift  0.5     (was 0.2)  slow EKF drift is what the loop
                                         compensates; jitter is still gated
                                         by ekf_gate_std
New (BARO): baro_gain, baro_tau_s, baro_ground_effect_alt, baro_blend_s,
    baro_glitch_m, baro_stale_s, baro_stale_abort_s, baro_max_cmd_offset,
    baro_gate_std, baro_gate_drift, baro_min_rate_hz, ekf_gate_std
    — see takeoff_land_barometer/baro_altitude.py.

USAGE:
    ros2 launch takeoff_land_barometer takeoff_land_barometer.launch.py \
        target_alt:=2.0 hover_time:=8.0 max_pos_error:=0.5 max_vz:=1.0 max_alt_error:=0.5

START ORDER (with delays):
    t=0   : kill stale processes + start MAVROS
    t=10s : start px4_sensor_reader
    t=14s : start takeoff_land_baro_node
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

from takeoff_land_barometer.launch_common import (
    takeoff_land_arguments, takeoff_land_parameters, mavros_and_reader)


def generate_launch_description():
    args = takeoff_land_arguments()
    flight_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="takeoff_land_barometer",
                executable="takeoff_land_baro_node",
                name="takeoff_land_baro_node",
                output="screen",
                parameters=[takeoff_land_parameters()],
            ),
        ],
    )
    return LaunchDescription(args + mavros_and_reader() + [flight_node])
