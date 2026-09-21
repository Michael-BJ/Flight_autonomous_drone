#!/usr/bin/env python3
"""
fm_all_barometer.launch.py
==========================
One terminal for everything, like fm_deploy's fm_all.launch.py: MAVROS +
Orbbec Gemini 2 camera driver + the fm stack — but the inference node is
fm_deploy_barometer's (barometric altitude hold + return-along-the-flown-
path recovery; use_baro:=false for EKF altitude + recovery only).

COPIED FROM fm_deploy/launch/fm_all.launch.py on 2026-09-13 — when that
file changes, bring the change here. Lines that differ are tagged BARO.
Camera arguments are identical (depth only, publish_tf:=false, ...).

Example:
    ros2 launch fm_deploy_barometer fm_all_barometer.launch.py \\
        goal_dist:=3.0 target_alt:=2.0 max_alt_error:=0.5 dry_run:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,     # RTHYAWMAP
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition                              # RTHYAWMAP
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node                                     # MAPVIEW
from launch_ros.parameter_descriptions import ParameterValue           # MAPVIEW


def _f(name):                                                          # MAPVIEW
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _i(name):                                                          # MAPVIEW
    return ParameterValue(LaunchConfiguration(name), value_type=int)

_FM_DEPLOY_SHARE = get_package_share_directory("fm_deploy")
_FM_BARO_SHARE   = get_package_share_directory("fm_deploy_barometer")   # BARO
_ORBBEC_SHARE    = get_package_share_directory("orbbec_camera")

_DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037_trt_fp32.engine")

# Arguments forwarded 1:1 to fm_real_barometer.launch.py. (name, default)
_FORWARDED = [
    # BARO
    ("use_baro",            "true"),
    # model
    ("model_path",          _DEFAULT_MODEL_PATH),
    ("K",                   "8"),
    ("onnx_fallback_on_oom", "true"),
    ("onnx_fallback_path",  ""),
    # mission
    ("goal_dist",           "5.0"),
    ("goal_lat",            "0.0"),
    ("target_alt",          "2.0"),      # BARO (fm_all: 1.5)
    ("v_max",               "0.5"),
    ("mission_timeout_s",   "180.0"),
    ("hover_settle_s",      "5.0"),
    # monitoring
    ("status_period_s",     "2.0"),
    ("color_output",        "true"),
    ("warn_replan_overrun", "true"),
    # safety / planner (defaults copied from fm_all.launch.py)
    ("use_safety_guards",   "false"),
    ("use_lookahead_guard", "true"),
    ("guard_clearance",     "0.60"),
    ("hard_clearance",      "0.55"),
    ("collision_cost_tol",  "20.0"),
    ("max_plan_cost",       "50.0"),
    ("planning_time_ahead", "0.3"),
    ("speed_margin_k",      "0.25"),
    ("use_speed_limit",     "true"),
    ("blind_abort_s",       "10.0"),
    ("stuck_abort_s",       "60.0"),
    ("depth_max_lag",       "0.5"),
    ("max_candidates",      "1"),        # NEW (2026-09-14), same as fm_all
    ("replan_budget_s",     "1.0"),      # NEW (2026-09-14), same as fm_all
    ("w_feasibility",       "1000.0"),   # NEW (2026-09-15, WFEAS)
    ("flying_yaw_mode",     "home"),     # NEW (2026-09-15, YAWSMOOTH)
    ("yaw_smooth_tau_s",    "1.0"),      # NEW (2026-09-15, YAWSMOOTH)
    ("yaw_rate_max_dps",    "30.0"),     # NEW (2026-09-15, YAWSMOOTH)
    ("yaw_min_speed",       "0.15"),     # NEW (2026-09-15, YAWSMOOTH)
    ("yaw_max_offset_deg",  "60.0"),     # NEW (2026-09-15, YAWSMOOTH)
    ("yaw_deadband_deg",    "15.0"),     # NEW (2026-09-15, YAWSMOOTH)
    ("min_battery_pct",     "0.0"),
    ("rc_override_enabled", "true"),
    ("verify_rc_override_param", "true"),
    ("write_px4_params",    "false"),
    ("px4_vel_cap",         "2.0"),
    ("descent_speed",       "0.3"),
    ("land_handoff_alt",    "0.25"),
    ("auto_land_mode",      "true"),
    ("min_fix_type",        "3"),
    ("gps_wait_timeout",    "120.0"),
    ("gps_stable_dur",      "5.0"),
    ("ekf_window_s",        "5.0"),
    ("max_ground_drift",    "0.5"),      # BARO (fm_all: 0.2)
    ("ekf_gate_std",        "0.15"),     # BARO
    ("dry_run",             "true"),
    ("safe_dis",            "0.8"),
    ("fence_fwd",           "10.0"),
    ("fence_back",          "3.0"),
    ("fence_lat",           "4.0"),
    ("max_home_dist",       "12.0"),
    ("max_alt_error",       "0.5"),
    ("min_battery_v",       "0.0"),
    ("require_rc_offboard", "true"),
    ("require_gps",         "true"),
    ("min_satellites",      "8"),
    ("max_hdop",            "2.0"),
    ("max_ground_z",        "1000.0"),   # BARO (fm_all: 1.0)
    ("ekf_pre_wait_s",      "10.0"),
    ("cam_x",               "0.10"),
    ("cam_y",               "0.00"),
    ("cam_z",               "-0.05"),
    ("gate_enabled",        "true"),
    ("gate_alt_margin",     "1.5"),      # BARO (fm_all: 0.5)
    # BARO: barometric hold
    ("baro_gain",              "0.0"),   # NEW (2026-09-15, GAIN0): was 0.7
    ("baro_tau_s",             "1.0"),
    ("baro_ground_effect_alt", "1.0"),
    ("baro_blend_s",           "2.0"),
    ("baro_glitch_m",          "1.5"),
    ("baro_stale_s",           "1.0"),
    ("baro_stale_abort_s",     "5.0"),
    ("baro_max_cmd_offset",    "1.5"),
    ("baro_gate_std",          "0.08"),
    ("baro_gate_drift",        "0.30"),
    ("baro_min_rate_hz",       "5.0"),
    ("band_drift_margin",      "0.5"),
    # BARO: return recovery
    ("rth_enabled",         "true"),
    ("rth_speed",           "0.3"),
    ("rth_crumb_spacing",   "0.3"),
    ("rth_min_dist",        "1.0"),
    ("rth_arrive_tol",      "0.5"),
    ("rth_lead_max",        "0.6"),
    ("rth_max_track_err",   "1.5"),
    ("rth_track_err_s",     "3.0"),
    ("rth_min_clearance",   "0.50"),
    ("rth_block_s",         "8.0"),
    ("rth_yaw_rate_dps",    "45.0"),
    ("rth_timeout_extra_s", "30.0"),
    ("rth_hover_s",         "2.0"),
    ("rth_after_goal",      "false"),   # NEW (2026-09-16, RTHGOAL)
    ("rth_mode",            "trail"),   # NEW (2026-09-16, RTHREPLAN)
    ("rth_replan_timeout_s", "0.0"),
    ("publish_3d_map",      "false"),   # NEW (2026-09-16, RTHYAWMAP)
]

# Camera arguments (identical to fm_all.launch.py — depth only, no TF).
_CAMERA = [
    ("enable_color",       "false"),
    ("enable_ir",          "false"),
    ("enable_point_cloud", "false"),
    ("depth_registration", "false"),
    ("depth_width",        "0"),
    ("depth_height",       "0"),
    ("depth_fps",          "0"),
    ("publish_tf",         "false"),
    ("enable_sync_output_accel_gyro", "false"),
    ("enable_accel",       "false"),
    ("enable_gyro",        "false"),
]


def _stamp():
    """Timestamp for the rosbag directory name (RTHYAWMAP)."""
    import datetime
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyACM0:57600",
            description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600"),
    ]
    args += [DeclareLaunchArgument(n, default_value=d) for n, d in _FORWARDED]
    args += [DeclareLaunchArgument(n, default_value=d) for n, d in _CAMERA]
    # NEW (2026-09-16, RTHYAWMAP): octomap recording for post-flight debugging.
    args += [
        DeclareLaunchArgument(
            "record_map", default_value="false",
            description="true: record the octomap + pose into a rosbag under "
                        "map_record_dir (set publish_3d_map:=true for the 3D map)"),
        DeclareLaunchArgument(
            "map_record_dir",
            default_value=os.path.expanduser("~/flight_maps"),
            description="directory the rosbag of record_map:=true is written to"),
    ]
    # NEW (2026-09-21, MAPVIEW): live ASCII map in the terminal, for SSH.
    args += [
        DeclareLaunchArgument(
            "view_map", default_value="false",
            description="true: draw /projected_map as an ASCII map in this "
                        "terminal (read-only node, publishes nothing)"),
        DeclareLaunchArgument(
            "view_hz", default_value="0.5",
            description="map redraw rate (Hz). Keep it low here — the map "
                        "shares this terminal with the mission log"),
        DeclareLaunchArgument(
            "view_rows", default_value="18",
            description="map height in text rows (0 = size of the terminal)"),
        DeclareLaunchArgument(
            "view_cols", default_value="90",
            description="map width in text columns (0 = size of the terminal)"),
        DeclareLaunchArgument(
            "view_range", default_value="0.0",
            description="0 = fit the whole map; >0 = +/- this many metres "
                        "around the drone"),
        DeclareLaunchArgument(
            "view_mode", default_value="scroll",
            description="scroll: print a block each refresh (safe with other "
                        "logs) | fullscreen: redraw in place (own terminal)"),
    ]

    mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{_FM_DEPLOY_SHARE}/launch/mavros_only.launch.py"),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
        }.items(),
    )

    camera = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{_ORBBEC_SHARE}/launch/gemini2.launch.py"),
                launch_arguments={
                    n: LaunchConfiguration(n) for n, _ in _CAMERA
                }.items(),
            ),
        ],
    )

    fm_stack = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{_FM_BARO_SHARE}/launch/fm_real_barometer.launch.py"),   # BARO
                launch_arguments={
                    n: LaunchConfiguration(n) for n, _ in _FORWARDED
                }.items(),
            ),
        ],
    )

    # NEW (2026-09-16, RTHYAWMAP): listens only — it publishes nothing and
    # never touches /dev/ttyACM0. Topics that do not exist yet are picked up
    # as soon as octomap_server starts.
    map_recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_map")),
        cmd=["ros2", "bag", "record", "-o",
             [LaunchConfiguration("map_record_dir"), "/map_",
              _stamp()],
             "/projected_map", "/octomap_binary", "/octomap_full",
             "/mavros/local_position/pose", "/mavros/local_position/odom",
             "/tf", "/tf_static"],
        output="screen",
    )

    # NEW (2026-09-21, MAPVIEW): subscribes to /projected_map + /px4/sensors +
    # /px4/state and prints them. It publishes nothing, calls no service and
    # never opens /dev/ttyACM0, so it cannot affect a flight. Started after
    # the fm stack so the mission banner is not buried under map frames.
    map_view = TimerAction(
        period=14.0,
        actions=[
            Node(
                condition=IfCondition(LaunchConfiguration("view_map")),
                package="fm_deploy_barometer",
                executable="octomap_view_node",
                name="octomap_view_node",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "view_hz":    _f("view_hz"),
                    "view_mode":  LaunchConfiguration("view_mode"),
                    "view_rows":  _i("view_rows"),
                    "view_cols":  _i("view_cols"),
                    "view_range": _f("view_range"),
                    "view_color": "true",
                    # Same goal the mission flies to, so the G marker matches.
                    "goal_dist":  _f("goal_dist"),
                    "goal_lat":   _f("goal_lat"),
                }],
            ),
        ],
    )

    return LaunchDescription(
        args + [mavros, camera, fm_stack, map_recorder, map_view])
