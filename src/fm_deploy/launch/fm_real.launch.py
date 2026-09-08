#!/usr/bin/env python3
"""
fm_real.launch.py
=================
FM-Planner inference stack for the REAL DRONE (Jetson Orin Nano + PX4 + Gemini 2).
Real-world equivalent of `fm_planning_unknown.launch.py` in the simulation workspace.

Nodes launched:
    mavros_tf_broadcaster_node   /mavros/local_position/pose -> TF odom->base_link
    static_transform_publisher   base_link -> camera_depth_frame  (MUST BE MEASURED!)
    gemini2_depth_bridge_node    /camera/depth/image_raw -> /realsense/depth/float32
    depth_to_pointcloud_node     -> /realsense/depth/points
    octomap_server_unknown       -> /projected_map  (read by ESDF)
    fm_inference_real_node       FM + MINCO + hardware safety layer

TERMINAL ORDER (don't swap):
    T1  ros2 launch fm_deploy mavros_only.launch.py fcu_url:=/dev/ttyTHS1:921600
        -> wait for "Got HEARTBEAT" / [BAT] to populate
    T2  ros2 launch orbbec_camera gemini2.launch.py        # camera driver
        -> confirm: ros2 topic hz /camera/depth/image_raw
    T3  ros2 launch fm_deploy fm_real.launch.py dry_run:=true
        # model_path defaults to .onnx, may be omitted; override
        # model_path:=<path .pth> to force a different backend
    T4  (optional) rviz2   — view /projected_map + /planner/candidates

FIRST FLIGHT — DO NOT SKIP:
    1) dry_run:=true, PROPELLERS OFF. Confirm "[INF] Replan ok" shows up
       and the candidate markers look reasonable.
    2) dry_run:=false, goal_dist:=0.0, empty room -> takeoff/hover/land only.
    3) goal_dist:=3.0 with no obstacle.
    4) only then add an obstacle. Pilot holds the RC with the mode switch ready.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Default model_path: .onnx, not .pth — this Jetson's GPU still fails with
# CUBLAS_STATUS_ALLOC_FAILED (a JetPack 6.2/CUDA 12.6 driver bug, no fix as
# of 2026-08-10, see the onnxruntime-gpu-jetson memory), so .pth ALWAYS
# falls back to .onnx-CPU via the failed-GPU-attempt path first (wastes
# time + confusing error logs). Going straight to .onnx means the node
# skips the GPU attempt and goes straight to CPUExecutionProvider, same
# end result. Override model_path:=...pth on the command line once the
# GPU is fixed and you want to re-test it.
_DEFAULT_MODEL_PATH = os.path.expanduser(
    "~/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx")


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _b(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _i(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    args = [
        # ── Model ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "model_path", default_value=_DEFAULT_MODEL_PATH,
            description="FM checkpoint path (.onnx recommended on Jetson, or "
                        ".pth). Defaults to .onnx — see the "
                        "_DEFAULT_MODEL_PATH comment above."),
        DeclareLaunchArgument(
            "K", default_value="8",
            description="Number of FM candidates per replan. IMPORTANT: the "
                        "existing ONNX export locks noise to [8, 9], so with "
                        "the .onnx backend this value MUST be 8. Only .pth "
                        "is free to change it."),
        DeclareLaunchArgument("n_steps", default_value="2",
                              description="Euler steps (.pth backend only)"),
        DeclareLaunchArgument(
            "onnx_fallback_on_oom", default_value="true",
            description="true = if .pth fails to load due to CUDA "
                        "out-of-memory, automatically switch to .onnx (K "
                        "forced to 8) instead of crashing the node. Verified "
                        "necessary on this Jetson 2026-07-28 (unified memory "
                        "fragmentation)."),
        DeclareLaunchArgument(
            "onnx_fallback_path", default_value="",
            description="Explicit .onnx path to use for the fallback. Empty "
                        "= auto-derive from model_path (.pth -> .onnx)."),

        # ── Mission (HOME frame — see fm_inference_real_node.py) ──────────────
        DeclareLaunchArgument(
            "goal_dist", default_value="5.0",
            description="Goal distance AHEAD of the home point (m). 0 = "
                        "takeoff/hover/land only (stage-2 test)."),
        DeclareLaunchArgument("goal_lat", default_value="0.0",
                              description="Shift the goal left(+)/right(-) (m)"),
        DeclareLaunchArgument("target_alt", default_value="1.5",
                              description="Cruise altitude above ground (m)"),
        DeclareLaunchArgument(
            "v_max", default_value="0.5",
            description="Max speed (m/s). START LOW. Simulation uses 1.0; "
                        "in a real room 0.4-0.6 is much safer."),
        DeclareLaunchArgument("replan_period", default_value="1.0"),
        DeclareLaunchArgument(
            "cmd_hz", default_value="50",
            description="Setpoint rate (Hz). 50 = same as takeoff_land, "
                        "already proven over serial; 100 floods the "
                        "telemetry link."),
        DeclareLaunchArgument("mission_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("hover_settle_s", default_value="5.0"),
        DeclareLaunchArgument("auto_reverse", default_value="false",
                              description="false = stop at the goal (do NOT "
                                          "set true for the first flight)"),

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
            description="Max z deviation from cruise_z before ABORT (m). "
                        "0.5 -> flight window target_alt +/- 0.5."),
        DeclareLaunchArgument("min_battery_v",   default_value="0.0",
                              description="Minimum voltage (V). 0 = disabled. "
                                          "4S LiPo: ~14.4 V is reasonable."),
        DeclareLaunchArgument("min_battery_pct", default_value="0.0"),
        # ── GPS quality pre-arm gate (see _wait_gps_quality in the node) ─────
        DeclareLaunchArgument(
            "require_gps", default_value="true",
            description="Refuse to ARM until GPS is genuinely usable "
                        "(fix_type/satellites/HDOP). Skipped automatically in "
                        "dry_run. Set false ONLY for indoor flight where a "
                        "healthy VIO/optical-flow source provides PX4's local "
                        "position."),
        DeclareLaunchArgument("min_fix_type",     default_value="3"),
        DeclareLaunchArgument("min_satellites",   default_value="8"),
        DeclareLaunchArgument("max_hdop",         default_value="2.0"),
        DeclareLaunchArgument("gps_wait_timeout", default_value="120.0"),
        DeclareLaunchArgument("gps_stable_dur",   default_value="5.0"),
        # ── EKF ground_z trust criteria (see _wait_ekf_stable in the node) ──
        DeclareLaunchArgument(
            "max_ground_z", default_value="1.0",
            description="Max |local-frame z| accepted while ON THE GROUND. A "
                        "steady but far-from-zero z means the estimate is "
                        "broken, not stable (2026-08-27: 5.46 m was accepted "
                        "and the drone hit a tree)."),
        DeclareLaunchArgument("ekf_window_s",     default_value="5.0"),
        DeclareLaunchArgument("max_ground_drift", default_value="0.20"),
        DeclareLaunchArgument("rc_override_enabled", default_value="true"),
        DeclareLaunchArgument(
            "verify_rc_override_param", default_value="true",
            description="Check COM_RC_OVERRIDE on PX4 before flying. false "
                        "is for bench testing only, with no RC bound."),
        DeclareLaunchArgument("descent_speed",    default_value="0.3"),
        DeclareLaunchArgument("land_handoff_alt", default_value="0.25"),
        DeclareLaunchArgument("auto_land_mode",   default_value="true"),
        DeclareLaunchArgument(
            "write_px4_params", default_value="false",
            description="true = the node writes MPC_XY_* / EKF2_HGT_REF. "
                        "Default false: don't let the node silently change "
                        "your FCU parameters."),
        DeclareLaunchArgument("px4_vel_cap", default_value="2.0",
                              description="Only used when write_px4_params:=true"),
        DeclareLaunchArgument(
            "ekf_pre_wait_s", default_value="10.0",
            description="Initial GPS/EKF convergence delay before the "
                        "ground_z std-check (same as takeoff_land_node.py)."),

        # ── Planner guards ───────────────────────────────────────────────────
        # ESCAPE MODE DISABLED (operator request, 2026-07-27).
        # use_safety_guards is the ONLY escape gate: all four
        # _enter_escape() trigger points in fm_inference_base.py (collision
        # guard, look-ahead guard, no-progress, and 2x failed replan) are
        # all wrapped in `if self._use_guards`. With it false + use_lookahead_guard
        # true, guards 1 & 2 STILL detect danger but their action becomes
        # "invalidate + hover" — the drone stops in place and replans right
        # away, instead of running an 8-direction escape probe.
        # If every replan keeps failing: the map auto-resets, then
        # stuck_abort_s (60s) / blind_abort_s (10s) takes over -> abort ->
        # controlled landing. There is no automatic rescue.
        DeclareLaunchArgument(
            "use_safety_guards", default_value="false",
            description="FALSE = escape mode OFF (guards 1&2 stay active, "
                        "their action is hover+replan). This is also the "
                        "'fair' mode used for ablation in simulation, so "
                        "real-flight results are directly comparable. Do "
                        "NOT set true unless you actually want escape "
                        "maneuvers back."),
        DeclareLaunchArgument(
            "use_lookahead_guard", default_value="true",
            description="MUST be true while use_safety_guards is false — "
                        "this is what leaves guards 1 & 2 in place. If both "
                        "are false, there is NO guard at all."),
        DeclareLaunchArgument(
            "safe_dis", default_value="0.8",
            description="Planner's soft preference (m from drone CENTER). "
                        "TEMPORARILY raised from the code default 0.65 -> "
                        "0.8 m."),
        DeclareLaunchArgument("hard_clearance",     default_value="0.55"),
        DeclareLaunchArgument("guard_clearance",    default_value="0.60"),
        DeclareLaunchArgument("collision_cost_tol", default_value="20.0"),
        DeclareLaunchArgument("planning_time_ahead", default_value="0.3"),
        DeclareLaunchArgument("speed_margin_k",     default_value="0.25"),
        DeclareLaunchArgument("max_plan_cost",      default_value="50.0"),
        DeclareLaunchArgument("use_speed_limit",    default_value="true"),
        DeclareLaunchArgument("blind_abort_s",      default_value="10.0"),
        DeclareLaunchArgument("stuck_abort_s",      default_value="60.0"),
        DeclareLaunchArgument("depth_max_lag",      default_value="0.5"),
        DeclareLaunchArgument("use_mode_persistence",   default_value="false"),
        DeclareLaunchArgument("use_bimodal_steering",   default_value="false"),
        DeclareLaunchArgument("use_anchor_sampling",    default_value="false"),
        DeclareLaunchArgument("use_efficiency_ranking", default_value="false"),
        DeclareLaunchArgument("candidate_log_path",     default_value=""),
        DeclareLaunchArgument("publish_markers",        default_value="true"),
        # ── Terminal monitoring (see fm_inference_real_node._announce_phase) ──
        DeclareLaunchArgument(
            "status_period_s", default_value="2.0",
            description="How often the [T+mm:ss] PHASE status line is printed."),
        DeclareLaunchArgument(
            "color_output", default_value="true",
            description="ANSI colour in the terminal. Set false when piping "
                        "the log to a file."),
        DeclareLaunchArgument(
            "warn_replan_overrun", default_value="true",
            description="Warn when a replan takes longer than "
                        "planning_time_ahead — that parameter IS the "
                        "compute-time budget, and overrunning it installs a "
                        "trajectory the drone has already passed."),

        # ── Camera & map ─────────────────────────────────────────────────────
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_info_topic",
                              default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("depth_scale", default_value="0.001",
                              description="16UC1 -> meters (Gemini 2: 1 mm/unit)"),
        DeclareLaunchArgument(
            "invalid_fill_m", default_value="10.0",
            description="Value for invalid depth pixels. 10.0 = 'far/safe' "
                        "(training-data convention). DO NOT use 0.0 — that "
                        "means 'obstacle touching the lens'."),
        DeclareLaunchArgument("hole_fill_px", default_value="5"),
        DeclareLaunchArgument("resolution", default_value="0.15",
                              description="Octomap resolution (m)"),
        DeclareLaunchArgument("max_range",  default_value="4.0",
                              description="Effective depth range (m)"),
        DeclareLaunchArgument("min_range",  default_value="0.5"),
        DeclareLaunchArgument("occ_min_z",  default_value="0.6"),
        DeclareLaunchArgument("occ_max_z",  default_value="2.0"),
        DeclareLaunchArgument("publish_3d_map", default_value="false"),

        # ── Altitude gate for depth processing ───────────────────────────────
        # Depth -> pointcloud back-projection (and therefore all octomap
        # insertion) only runs while the drone is within the band
        # target_alt +/- gate_alt_margin above ground. The depth image
        # itself (/realsense/depth/float32) is NOT gated, so RViz still
        # shows depth at any altitude.
        # IMPORTANT: the gate is deliberately OPEN until the drone has ever
        # reached that band. fm_inference_real_node waits for depth + ESDF
        # BEFORE ARM (step 4, 90s timeout); a gate closed on the ground
        # would make that step always fail. Ground voxels collected during
        # that window are discarded anyway since octomap resets after
        # takeoff settle (step 13).
        DeclareLaunchArgument(
            "gate_enabled", default_value="true",
            description="false = process depth at all altitudes (old behavior)"),
        DeclareLaunchArgument(
            "gate_alt_margin", default_value="0.5",
            description="Half-width of the band, meters. 0.5 -> 0.7..1.7 m "
                        "for target_alt 1.2. The band's center tracks "
                        "target_alt."),

        # ── Camera mount on the drone body — MEASURE IT YOURSELF! ────────────
        # cam_x forward(+), cam_y left(+), cam_z up(+) from base_link center.
        # roll/pitch/yaw rotate base_link (FLU) -> the camera's OPTICAL
        # frame (z forward, x right, y down). Being off by a few
        # centimeters here shifts the ENTIRE map -> the drone dodges toward
        # the wrong place.
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
            # Gate band center = same target_alt as the planner.
            "gate_enabled":    _b("gate_enabled"),
            "gate_target_alt": _f("target_alt"),
            "gate_alt_margin": _f("gate_alt_margin"),
            "gate_pose_topic": "/mavros/local_position/pose",
            # Ground-gate latch window = the planner's EKF convergence
            # window, so both nodes agree on where "ground" is.
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

    fm_node = Node(
        package="fm_deploy",
        executable="fm_inference_real_node",
        name="fm_inference_real_node",
        output="screen",
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
