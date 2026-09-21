#!/usr/bin/env python3
"""
fm_real_barometer.launch.py
===========================
fm_deploy's fm_real.launch.py with the inference node replaced by
fm_deploy_barometer's:

    use_baro:=true  (default)  fm_inference_baro_node
                               = barometric altitude hold + RETURN recovery
    use_baro:=false            fm_inference_recovery_node
                               = EKF/GPS altitude (as fm_deploy) + RETURN recovery

Everything else (TF broadcaster, camera TF, depth bridge, point cloud,
octomap) is the same node set with the same arguments as fm_real.launch.py.
COPIED FROM fm_deploy/launch/fm_real.launch.py on 2026-09-13 — when that
file changes, bring the change here. Lines that differ are tagged BARO.

Defaults that differ from fm_real.launch.py (BARO):
    target_alt        2.0    (was 1.5)   the octomap band is only tolerant of
                                         EKF drift when the drone flies higher
    max_ground_z      1000.0 (was 1.0)   absolute EKF z is irrelevant to the
                                         barometric loop
    max_ground_drift  0.5    (was 0.2)   slow EKF drift is compensated
    gate_alt_margin   1.5    (was 0.5)   the depth gate reads the EKF altitude,
                                         which is now EXPECTED to drift

TERMINAL ORDER / FIRST FLIGHT: see fm_real.launch.py. Start with
dry_run:=true (default), then goal_dist:=0.0, then a short goal.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Same default as fm_deploy (TensorRT engine, automatic .onnx/CPU fallback).
_DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037_trt_fp32.engine")


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    args = [
        # ── BARO: which inference node ───────────────────────────────────────
        DeclareLaunchArgument(
            "use_baro", default_value="true",
            description="true = fm_inference_baro_node (barometric altitude + "
                        "return recovery). false = fm_inference_recovery_node "
                        "(EKF/GPS altitude like fm_deploy + return recovery)."),
        # ── Model ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "model_path", default_value=_DEFAULT_MODEL_PATH,
            description="FM model path (.engine = TensorRT GPU, .onnx or "
                        ".pth). Defaults to the TensorRT .engine."),
        DeclareLaunchArgument(
            "K", default_value="8",
            description="Number of FM candidates per replan. With the .onnx/"
                        ".engine backend this value MUST be 8."),
        DeclareLaunchArgument("n_steps", default_value="2",
                              description="Euler steps (.pth backend only)"),
        DeclareLaunchArgument("onnx_fallback_on_oom", default_value="true"),
        DeclareLaunchArgument("onnx_fallback_path", default_value=""),

        # ── Mission (HOME frame — see fm_inference_real_node.py) ──────────────
        DeclareLaunchArgument(
            "goal_dist", default_value="5.0",
            description="Goal distance AHEAD of the home point (m). 0 = "
                        "takeoff/hover/land only (stage-2 test)."),
        DeclareLaunchArgument("goal_lat", default_value="0.0",
                              description="Shift the goal left(+)/right(-) (m)"),
        DeclareLaunchArgument(
            "target_alt", default_value="2.0",   # BARO (fm_real: 1.5)
            description="Cruise altitude above ground (m), BAROMETRIC. >= 2.0 "
                        "recommended: the octomap band [alt-0.7, alt+1.5] is "
                        "in the drifting EKF frame."),
        DeclareLaunchArgument(
            "v_max", default_value="0.5",
            description="Max speed (m/s). START LOW."),
        DeclareLaunchArgument("replan_period", default_value="1.0"),
        DeclareLaunchArgument("cmd_hz", default_value="50"),
        DeclareLaunchArgument("mission_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("hover_settle_s", default_value="5.0"),
        DeclareLaunchArgument("auto_reverse", default_value="false"),

        # ── Safety ───────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="DEFAULT TRUE. Full pipeline WITHOUT arming & "
                        "WITHOUT setpoints. Only set false after a clean "
                        "dry run."),
        DeclareLaunchArgument("fence_fwd",  default_value="10.0"),
        DeclareLaunchArgument("fence_back", default_value="3.0"),
        DeclareLaunchArgument("fence_lat",  default_value="4.0"),
        DeclareLaunchArgument("max_home_dist", default_value="12.0",
                              description="Hard abort radius from home (m)"),
        DeclareLaunchArgument(
            "max_alt_error", default_value="0.5",
            description="Max BAROMETRIC altitude deviation from target_alt "
                        "before ABORT (m)."),
        DeclareLaunchArgument("min_battery_v",   default_value="0.0"),
        DeclareLaunchArgument("min_battery_pct", default_value="0.0"),
        DeclareLaunchArgument("require_rc_offboard", default_value="true"),
        DeclareLaunchArgument("require_gps", default_value="true"),
        DeclareLaunchArgument("min_fix_type",     default_value="3"),
        DeclareLaunchArgument("min_satellites",   default_value="8"),
        DeclareLaunchArgument("max_hdop",         default_value="2.0"),
        DeclareLaunchArgument("gps_wait_timeout", default_value="120.0"),
        DeclareLaunchArgument("gps_stable_dur",   default_value="5.0"),
        # ── EKF criteria on the ground (BARO defaults) ───────────────────────
        DeclareLaunchArgument(
            "max_ground_z", default_value="1000.0",   # BARO (fm_real: 1.0)
            description="Max |EKF z| on the ground. Irrelevant to the "
                        "barometric loop, effectively disabled."),
        DeclareLaunchArgument("ekf_window_s",     default_value="5.0"),
        DeclareLaunchArgument(
            "max_ground_drift", default_value="0.5",  # BARO (fm_real: 0.2)
            description="Max EKF z peak-to-peak over ekf_window_s on the "
                        "ground; slow drift is compensated by the loop."),
        DeclareLaunchArgument(
            "ekf_gate_std", default_value="0.15",
            description="BARO: max EKF z std on the ground (jitter). PX4 still "
                        "flies on its own vertical velocity."),
        DeclareLaunchArgument("rc_override_enabled", default_value="true"),
        DeclareLaunchArgument("verify_rc_override_param", default_value="true"),
        DeclareLaunchArgument("descent_speed",    default_value="0.3"),
        DeclareLaunchArgument("land_handoff_alt", default_value="0.25"),
        DeclareLaunchArgument("auto_land_mode",   default_value="true"),
        DeclareLaunchArgument(
            "write_px4_params", default_value="false",
            description="Keep false. The barometric hold needs NO PX4 change."),
        DeclareLaunchArgument("px4_vel_cap", default_value="2.0"),
        DeclareLaunchArgument("ekf_pre_wait_s", default_value="10.0"),

        # ── BARO: barometric altitude hold (see baro_altitude.py) ───────────
        DeclareLaunchArgument("baro_gain",              default_value="0.0"),  # NEW (2026-09-15, GAIN0): was 0.7
        DeclareLaunchArgument("baro_tau_s",             default_value="1.0"),
        DeclareLaunchArgument("baro_ground_effect_alt", default_value="1.0"),
        DeclareLaunchArgument("baro_blend_s",           default_value="2.0"),
        DeclareLaunchArgument("baro_glitch_m",          default_value="1.5"),
        DeclareLaunchArgument("baro_stale_s",           default_value="1.0"),
        DeclareLaunchArgument("baro_stale_abort_s",     default_value="5.0"),
        DeclareLaunchArgument("baro_max_cmd_offset",    default_value="1.5"),
        DeclareLaunchArgument("baro_gate_std",          default_value="0.08"),
        DeclareLaunchArgument("baro_gate_drift",        default_value="0.30"),
        DeclareLaunchArgument("baro_min_rate_hz",       default_value="5.0"),
        DeclareLaunchArgument(
            "band_drift_margin", default_value="0.5",
            description="BARO: octomap band widened UPWARD by this (m) to "
                        "tolerate EKF drift during the flight."),

        # ── BARO: return-along-the-flown-path recovery (see recovery.py) ─────
        DeclareLaunchArgument(
            "rth_enabled", default_value="true",
            description="Recoverable planner stops (off-map / stuck / mission "
                        "timeout) return along the flown trail and land at "
                        "home. false = land in place as fm_deploy does."),
        DeclareLaunchArgument("rth_speed",           default_value="0.3"),
        DeclareLaunchArgument("rth_crumb_spacing",   default_value="0.3"),
        DeclareLaunchArgument("rth_min_dist",        default_value="1.0"),
        DeclareLaunchArgument("rth_arrive_tol",      default_value="0.5"),
        DeclareLaunchArgument("rth_lead_max",        default_value="0.6"),
        DeclareLaunchArgument("rth_max_track_err",   default_value="1.5"),
        DeclareLaunchArgument("rth_track_err_s",     default_value="3.0"),
        DeclareLaunchArgument("rth_min_clearance",   default_value="0.50"),
        DeclareLaunchArgument("rth_block_s",         default_value="8.0"),
        DeclareLaunchArgument("rth_yaw_rate_dps",    default_value="45.0"),
        DeclareLaunchArgument("rth_timeout_extra_s", default_value="30.0"),
        DeclareLaunchArgument("rth_hover_s",         default_value="2.0"),
        # NEW (2026-09-16, RTHGOAL): return home after a SUCCESSFUL mission too
        DeclareLaunchArgument("rth_after_goal",      default_value="false"),
        # NEW (2026-09-16, RTHREPLAN): "trail" (retrace) or "replan" (FM planner)
        DeclareLaunchArgument("rth_mode",            default_value="trail"),
        DeclareLaunchArgument("rth_replan_timeout_s", default_value="0.0"),

        # ── Planner guards (identical to fm_real.launch.py) ──────────────────
        DeclareLaunchArgument("use_safety_guards",   default_value="false"),
        DeclareLaunchArgument("use_lookahead_guard", default_value="true"),
        DeclareLaunchArgument("safe_dis",            default_value="0.8"),
        DeclareLaunchArgument("hard_clearance",      default_value="0.55"),
        DeclareLaunchArgument("guard_clearance",     default_value="0.60"),
        DeclareLaunchArgument("collision_cost_tol",  default_value="20.0"),
        DeclareLaunchArgument("planning_time_ahead", default_value="0.3"),
        DeclareLaunchArgument("speed_margin_k",      default_value="0.25"),
        DeclareLaunchArgument("max_plan_cost",       default_value="50.0"),
        DeclareLaunchArgument("use_speed_limit",     default_value="true"),
        DeclareLaunchArgument("blind_abort_s",       default_value="10.0"),
        DeclareLaunchArgument("stuck_abort_s",       default_value="60.0"),
        DeclareLaunchArgument("depth_max_lag",       default_value="0.5"),
        DeclareLaunchArgument("max_candidates",      default_value="1"),    # NEW (2026-09-14)
        DeclareLaunchArgument("replan_budget_s",     default_value="1.0"),  # NEW (2026-09-14)
        DeclareLaunchArgument("w_feasibility",       default_value="1000.0"),  # NEW (2026-09-15, WFEAS)
        # NEW (2026-09-15, YAWSMOOTH): yaw while FLYING (same as fm_real)
        DeclareLaunchArgument("flying_yaw_mode",     default_value="home"),
        DeclareLaunchArgument("yaw_smooth_tau_s",    default_value="1.0"),
        DeclareLaunchArgument("yaw_rate_max_dps",    default_value="30.0"),
        DeclareLaunchArgument("yaw_min_speed",       default_value="0.15"),
        DeclareLaunchArgument("yaw_max_offset_deg",  default_value="60.0"),
        DeclareLaunchArgument("yaw_deadband_deg",    default_value="15.0"),
        DeclareLaunchArgument("use_mode_persistence",   default_value="false"),
        DeclareLaunchArgument("use_bimodal_steering",   default_value="false"),
        DeclareLaunchArgument("use_anchor_sampling",    default_value="false"),
        DeclareLaunchArgument("use_efficiency_ranking", default_value="false"),
        DeclareLaunchArgument("candidate_log_path",     default_value=""),
        DeclareLaunchArgument("publish_markers",        default_value="true"),
        # ── Terminal monitoring ─────────────────────────────────────────────
        DeclareLaunchArgument("status_period_s",     default_value="2.0"),
        DeclareLaunchArgument("color_output",        default_value="true"),
        DeclareLaunchArgument("warn_replan_overrun", default_value="true"),

        # ── Camera & map (identical to fm_real.launch.py) ────────────────────
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_info_topic",
                              default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument("invalid_fill_m", default_value="10.0"),
        DeclareLaunchArgument("hole_fill_px", default_value="5"),
        DeclareLaunchArgument("resolution", default_value="0.15"),
        DeclareLaunchArgument("max_range",  default_value="4.0"),
        DeclareLaunchArgument("min_range",  default_value="0.5"),
        DeclareLaunchArgument("occ_min_z",  default_value="0.6"),
        DeclareLaunchArgument("occ_max_z",  default_value="2.0"),
        DeclareLaunchArgument("publish_3d_map", default_value="false"),

        # ── Altitude gate for depth processing ───────────────────────────────
        DeclareLaunchArgument("gate_enabled", default_value="true"),
        DeclareLaunchArgument(
            "gate_alt_margin", default_value="1.5",   # BARO (fm_real: 0.5)
            description="Half-width of the depth-processing band around "
                        "target_alt, measured on the EKF altitude — which the "
                        "barometric loop lets drift. Wide on purpose."),

        # ── Camera mount on the drone body — MEASURE IT YOURSELF ─────────────
        DeclareLaunchArgument("cam_x",     default_value="0.10"),
        DeclareLaunchArgument("cam_y",     default_value="0.00"),
        DeclareLaunchArgument("cam_z",     default_value="-0.05"),
        DeclareLaunchArgument("cam_roll",  default_value="-1.5708"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw",   default_value="-1.5708"),
    ]

    mavros_tf = Node(
        package="fm_deploy",
        executable="mavros_tf_broadcaster_node",
        name="mavros_tf_broadcaster_real",
        output="screen",
        parameters=[{
            "pose_topic":   "/mavros/local_position/pose",
            "parent_frame": "odom",
            "child_frame":  "base_link",
        }],
    )

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_camera_depth_frame",
        arguments=[
            "--x",     LaunchConfiguration("cam_x"),
            "--y",     LaunchConfiguration("cam_y"),
            "--z",     LaunchConfiguration("cam_z"),
            "--roll",  LaunchConfiguration("cam_roll"),
            "--pitch", LaunchConfiguration("cam_pitch"),
            "--yaw",   LaunchConfiguration("cam_yaw"),
            "--frame-id",       "base_link",
            "--child-frame-id", "camera_depth_frame",
        ],
        output="screen",
    )

    depth_bridge = Node(
        package="fm_deploy",
        executable="gemini2_depth_bridge_node",
        name="gemini2_depth_bridge_node",
        output="screen",
        parameters=[{
            "input_topic":      LaunchConfiguration("depth_topic"),
            "input_info_topic": LaunchConfiguration("depth_info_topic"),
            "frame_id":         "camera_depth_frame",
            "depth_scale":      _f("depth_scale"),
            "invalid_fill_m":   _f("invalid_fill_m"),
            "hole_fill_px":     _i("hole_fill_px"),
        }],
    )

    depth_to_cloud = Node(
        package="fm_deploy",
        executable="depth_to_pointcloud_node",
        name="depth_to_pointcloud",
        output="screen",
        parameters=[{
            "min_range":       _f("min_range"),
            "max_range":       _f("max_range"),
            "skip_pixels":     2,
            "gate_enabled":    _b("gate_enabled"),
            "gate_target_alt": _f("target_alt"),
            "gate_alt_margin": _f("gate_alt_margin"),
            "gate_pose_topic": "/mavros/local_position/pose",
            "gate_ground_settle_s": _f("ekf_pre_wait_s"),
        }],
    )

    # Node name MUST be "octomap_server_unknown": fm_inference_base calls
    # /octomap_server_unknown/set_parameters and /octomap_server_unknown/reset.
    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_unknown",
        output="screen",
        parameters=[{
            "frame_id":               "odom",
            "resolution":             _f("resolution"),
            "sensor_model.max_range": _f("max_range"),
            "publish_2d_map":         True,
            "publish_3d_map":         _b("publish_3d_map"),
            "queue_size":             50,
            "occupancy_min_z":        _f("occ_min_z"),
            "occupancy_max_z":        _f("occ_max_z"),
            "filter_ground_plane":    False,
            "use_height_map":         False,
            "latch":                  False,
        }],
        remappings=[("cloud_in", "/realsense/depth/points")],
    )

    # BARO: the inference node comes from fm_deploy_barometer
    fm_node = Node(
        package="fm_deploy_barometer",
        executable=PythonExpression([
            "'fm_inference_baro_node' if '", LaunchConfiguration("use_baro"),
            "'.lower() in ('true', '1') else 'fm_inference_recovery_node'"]),
        name="fm_inference_baro_node",
        output="screen",
        additional_env={"OPENBLAS_NUM_THREADS": "1"},   # NEW (2026-09-14), see fm_real.launch.py
        parameters=[{
            "model_path":          LaunchConfiguration("model_path"),
            "K":                   _i("K"),
            "n_steps":             _i("n_steps"),
            "onnx_fallback_on_oom": _b("onnx_fallback_on_oom"),
            "onnx_fallback_path":  LaunchConfiguration("onnx_fallback_path"),
            "goal_dist":           _f("goal_dist"),
            "goal_lat":            _f("goal_lat"),
            "use_home_frame":      True,
            "target_alt":          _f("target_alt"),
            "v_max":               _f("v_max"),
            "replan_period":       _f("replan_period"),
            "cmd_hz":              _i("cmd_hz"),
            "auto_reverse":        _b("auto_reverse"),
            "mission_timeout_s":   _f("mission_timeout_s"),
            "hover_settle_s":      _f("hover_settle_s"),
            # hardware safety
            "dry_run":             _b("dry_run"),
            "fence_fwd":           _f("fence_fwd"),
            "fence_back":          _f("fence_back"),
            "fence_lat":           _f("fence_lat"),
            "max_home_dist":       _f("max_home_dist"),
            "max_alt_error":       _f("max_alt_error"),
            "min_battery_v":       _f("min_battery_v"),
            "min_battery_pct":     _f("min_battery_pct"),
            "require_rc_offboard": _b("require_rc_offboard"),
            "require_gps":         _b("require_gps"),
            "min_fix_type":        _i("min_fix_type"),
            "min_satellites":      _i("min_satellites"),
            "max_hdop":            _f("max_hdop"),
            "gps_wait_timeout":    _f("gps_wait_timeout"),
            "gps_stable_dur":      _f("gps_stable_dur"),
            "max_ground_z":        _f("max_ground_z"),
            "ekf_window_s":        _f("ekf_window_s"),
            "max_ground_drift":    _f("max_ground_drift"),
            "rc_override_enabled": _b("rc_override_enabled"),
            "verify_rc_override_param": _b("verify_rc_override_param"),
            "descent_speed":       _f("descent_speed"),
            "land_handoff_alt":    _f("land_handoff_alt"),
            "auto_land_mode":      _b("auto_land_mode"),
            "write_px4_params":    _b("write_px4_params"),
            "px4_vel_cap":         _f("px4_vel_cap"),
            "ekf_pre_wait_s":      _f("ekf_pre_wait_s"),
            # BARO: barometric hold
            "ekf_gate_std":        _f("ekf_gate_std"),
            "baro_gain":           _f("baro_gain"),
            "baro_tau_s":          _f("baro_tau_s"),
            "baro_ground_effect_alt": _f("baro_ground_effect_alt"),
            "baro_blend_s":        _f("baro_blend_s"),
            "baro_glitch_m":       _f("baro_glitch_m"),
            "baro_stale_s":        _f("baro_stale_s"),
            "baro_stale_abort_s":  _f("baro_stale_abort_s"),
            "baro_max_cmd_offset": _f("baro_max_cmd_offset"),
            "baro_gate_std":       _f("baro_gate_std"),
            "baro_gate_drift":     _f("baro_gate_drift"),
            "baro_min_rate_hz":    _f("baro_min_rate_hz"),
            "band_drift_margin":   _f("band_drift_margin"),
            # BARO: return recovery
            "rth_enabled":         _b("rth_enabled"),
            "rth_speed":           _f("rth_speed"),
            "rth_crumb_spacing":   _f("rth_crumb_spacing"),
            "rth_min_dist":        _f("rth_min_dist"),
            "rth_arrive_tol":      _f("rth_arrive_tol"),
            "rth_lead_max":        _f("rth_lead_max"),
            "rth_max_track_err":   _f("rth_max_track_err"),
            "rth_track_err_s":     _f("rth_track_err_s"),
            "rth_min_clearance":   _f("rth_min_clearance"),
            "rth_block_s":         _f("rth_block_s"),
            "rth_yaw_rate_dps":    _f("rth_yaw_rate_dps"),
            "rth_timeout_extra_s": _f("rth_timeout_extra_s"),
            "rth_hover_s":         _f("rth_hover_s"),
            "rth_after_goal":      _b("rth_after_goal"),   # RTHGOAL
            "rth_mode":            LaunchConfiguration("rth_mode"),      # RTHREPLAN
            "rth_replan_timeout_s": _f("rth_replan_timeout_s"),
            # planner (inherited as-is from the simulation pipeline)
            "safe_dis":            _f("safe_dis"),
            "hard_clearance":      _f("hard_clearance"),
            "guard_clearance":     _f("guard_clearance"),
            "collision_cost_tol":  _f("collision_cost_tol"),
            "planning_time_ahead": _f("planning_time_ahead"),
            "speed_margin_k":      _f("speed_margin_k"),
            "max_plan_cost":       _f("max_plan_cost"),
            "use_speed_limit":     _b("use_speed_limit"),
            "blind_abort_s":       _f("blind_abort_s"),
            "stuck_abort_s":       _f("stuck_abort_s"),
            "depth_max_lag":       _f("depth_max_lag"),
            "max_candidates":      _i("max_candidates"),     # NEW (2026-09-14)
            "replan_budget_s":     _f("replan_budget_s"),    # NEW (2026-09-14)
            "w_feasibility":       _f("w_feasibility"),      # NEW (2026-09-15, WFEAS)
            # NEW (2026-09-15, YAWSMOOTH)
            "flying_yaw_mode":     LaunchConfiguration("flying_yaw_mode"),
            "yaw_smooth_tau_s":    _f("yaw_smooth_tau_s"),
            "yaw_rate_max_dps":    _f("yaw_rate_max_dps"),
            "yaw_min_speed":       _f("yaw_min_speed"),
            "yaw_max_offset_deg":  _f("yaw_max_offset_deg"),
            "yaw_deadband_deg":    _f("yaw_deadband_deg"),
            "use_safety_guards":   _b("use_safety_guards"),
            "use_lookahead_guard": _b("use_lookahead_guard"),
            "use_mode_persistence":   _b("use_mode_persistence"),
            "use_bimodal_steering":   _b("use_bimodal_steering"),
            "use_anchor_sampling":    _b("use_anchor_sampling"),
            "use_efficiency_ranking": _b("use_efficiency_ranking"),
            "candidate_log_path":  LaunchConfiguration("candidate_log_path"),
            "publish_markers":     _b("publish_markers"),
            "marker_frame":        "odom",
            "status_period_s":     _f("status_period_s"),
            "color_output":        _b("color_output"),
            "warn_replan_overrun": _b("warn_replan_overrun"),
        }],
    )

    return LaunchDescription(args + [
        mavros_tf, static_tf_camera, depth_bridge, depth_to_cloud,
        octomap, fm_node,
    ])
