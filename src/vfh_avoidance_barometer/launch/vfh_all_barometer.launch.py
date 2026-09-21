#!/usr/bin/env python3
"""
vfh_all_barometer.launch.py
===========================
All-in-one: MAVROS (serial) + px4_sensor_reader (takeoff_land)
+ Orbbec Gemini 2 depth camera + gemini2_depth_bridge_node (fm_deploy)
+ vfh_avoidance_node (perception) + vfh_flight_baro_node (flight).

TAKE OFF -> HOVER -> VFH-guided leg to a goal `goal_dist` m ahead
(steering around obstacles seen by the front depth camera) -> HOLD ->
LANDING where the drone ends up. Altitude from the BAROMETER (x/y from
GPS/EKF). PX4 parameters are NOT touched.

Arguments = takeoff_land_barometer.launch.py's (same names, same BARO
defaults) + forward_move's forward_speed / forward_hold_time + the VFH
flight and perception parameters (see launch_common.py) + the camera
arguments of fm_deploy/fm_all.launch.py (depth only, publish_tf false).

USAGE (first flights: short goal, slow, wide margins):
    ros2 launch vfh_avoidance_barometer vfh_all_barometer.launch.py \
        goal_dist:=3.0 forward_speed:=0.3 target_alt:=2.0 \
        max_pos_error:=1.0 max_vz:=1.0 max_alt_error:=0.5

START ORDER (with delays):
    t=0   : kill stale processes + start MAVROS
    t=5s  : Orbbec Gemini 2 driver (depth only)
    t=8s  : gemini2_depth_bridge_node
    t=9s  : vfh_avoidance_node
    t=10s : px4_sensor_reader
    t=14s : vfh_flight_baro_node  (refuses to arm until /vfh/movement_direction
                                   is alive at >= vfh_min_rate_hz)
"""
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node

from takeoff_land_barometer.launch_common import (
    takeoff_land_arguments, takeoff_land_parameters, mavros_and_reader, _f)
from vfh_avoidance_barometer.launch_common import (
    camera_arguments, camera_include, depth_bridge_node,
    vfh_arguments, vfh_node, vfh_flight_arguments, vfh_flight_parameters)

# takeoff_land_barometer's _kill_stale does not know these names.
_OWN_NODES = ("vfh_flight_baro_node", "vfh_avoidance_node",
              "gemini2_depth_bridge_node", "orbbec_camera_node")


def _kill_own(context):
    for n in _OWN_NODES:
        try:
            subprocess.run(["pkill", "-9", "-f", n], capture_output=True)
        except Exception:
            pass
    return []


def generate_launch_description():
    # takeoff_land's target_alt default is 1.2 m; with the level front camera
    # the ground fills the lower ROI within d_max below ~2 m (see
    # vfh_avoidance_node.py), so this package defaults to 2.0 m.
    args = [a for a in takeoff_land_arguments() if a.name != "target_alt"] + [
        DeclareLaunchArgument(
            "target_alt", default_value="2.0",
            description="Cruise height above the take-off point (m, barometric). "
                        "Keep >= 2.0 for VFH (ground in the camera view below that)."),
        DeclareLaunchArgument(
            "forward_speed", default_value="0.3",
            description="Cruise setpoint speed along the heading in m/s (0.05-1.0)."),
        DeclareLaunchArgument(
            "forward_hold_time", default_value="3.0",
            description="Seconds to hold at the end of the leg before landing there."),
    ] + vfh_flight_arguments() + vfh_arguments() + camera_arguments()

    params = takeoff_land_parameters()
    params.update({
        "forward_speed":     _f("forward_speed"),
        "forward_hold_time": _f("forward_hold_time"),
    })
    params.update(vfh_flight_parameters())

    camera = TimerAction(period=5.0, actions=[camera_include()])
    bridge = TimerAction(period=8.0, actions=[depth_bridge_node()])
    vfh    = TimerAction(period=9.0, actions=[vfh_node()])
    flight = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="vfh_avoidance_barometer",
                executable="vfh_flight_baro_node",
                name="vfh_flight_baro_node",
                output="screen",
                parameters=[params],
            ),
        ],
    )
    return LaunchDescription(
        args + [OpaqueFunction(function=_kill_own)] + mavros_and_reader()
        + [camera, bridge, vfh, flight])
