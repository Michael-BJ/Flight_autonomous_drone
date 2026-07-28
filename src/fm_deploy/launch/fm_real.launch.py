#!/usr/bin/env python3
"""
fm_real.launch.py
=================
Stack inference FM-Planner untuk DRONE NYATA (Jetson Orin Nano + PX4 + Gemini 2).
Padanan real-world dari `fm_planning_unknown.launch.py` di workspace simulasi.

Node yang dijalankan:
    mavros_tf_broadcaster_node   /mavros/local_position/pose -> TF odom->base_link
    static_transform_publisher   base_link -> camera_depth_frame  (WAJIB DIUKUR!)
    gemini2_depth_bridge_node    /camera/depth/image_raw -> /realsense/depth/float32
    depth_to_pointcloud_node     -> /realsense/depth/points
    octomap_server_unknown       -> /projected_map  (dibaca ESDF)
    fm_inference_real_node       FM + MINCO + lapisan keselamatan hardware

URUTAN TERMINAL (jangan ditukar):
    T1  ros2 launch fm_deploy mavros_only.launch.py fcu_url:=/dev/ttyTHS1:921600
        -> tunggu log "Got HEARTBEAT" / [BAT] terisi
    T2  ros2 launch orbbec_camera gemini2.launch.py        # driver kamera
        -> pastikan: ros2 topic hz /camera/depth/image_raw
    T3  ros2 launch fm_deploy fm_real.launch.py \
            model_path:=<path .onnx atau .pth> dry_run:=true
    T4  (opsional) rviz2   — lihat /projected_map + /planner/candidates

PENERBANGAN PERTAMA — JANGAN LEWATI:
    1) dry_run:=true, PROPELLER DILEPAS. Pastikan "[INF] Replan ok" muncul
       dan marker kandidat masuk akal.
    2) dry_run:=false, goal_dist:=0.0, ruangan kosong -> hanya takeoff/hover/land.
    3) goal_dist:=3.0 tanpa obstacle.
    4) baru tambahkan obstacle. Pilot memegang RC dengan mode switch siap.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "model_path",
            description="Path checkpoint FM (.onnx disarankan di Jetson, atau .pth)"),
        DeclareLaunchArgument(
            "K", default_value="8",
            description="Jumlah kandidat FM per replan. PENTING: export ONNX "
                        "yang ada mengunci noise pada [8, 9], jadi dengan "
                        "backend .onnx nilai ini HARUS 8. Hanya .pth yang bebas."),
        DeclareLaunchArgument("n_steps", default_value="2",
                              description="Langkah Euler (khusus backend .pth)"),
        DeclareLaunchArgument(
            "onnx_fallback_on_oom", default_value="true",
            description="true = kalau .pth gagal dimuat karena CUDA "
                        "out-of-memory, otomatis pindah ke .onnx (K dipaksa "
                        "8) alih-alih node crash. Terverifikasi perlu di "
                        "Jetson ini 2026-07-28 (fragmentasi memori unified)."),
        DeclareLaunchArgument(
            "onnx_fallback_path", default_value="",
            description="Path .onnx eksplisit untuk fallback. Kosong = "
                        "derive otomatis dari model_path (.pth -> .onnx)."),

        # ── Misi (frame HOME — lihat fm_inference_real_node.py) ──────────────
        DeclareLaunchArgument(
            "goal_dist", default_value="5.0",
            description="Jarak goal ke DEPAN dari titik home (m). 0 = hanya "
                        "takeoff/hover/land (uji tahap 2)."),
        DeclareLaunchArgument("goal_lat", default_value="0.0",
                              description="Geser goal ke kiri(+)/kanan(-) (m)"),
        DeclareLaunchArgument("target_alt", default_value="1.5",
                              description="Ketinggian jelajah di atas tanah (m)"),
        DeclareLaunchArgument(
            "v_max", default_value="0.5",
            description="Kecepatan maksimum (m/s). MULAI RENDAH. Simulasi "
                        "memakai 1.0; di ruangan nyata 0.4-0.6 jauh lebih aman."),
        DeclareLaunchArgument("replan_period", default_value="1.0"),
        DeclareLaunchArgument(
            "cmd_hz", default_value="50",
            description="Laju setpoint (Hz). 50 = sama dengan takeoff_land "
                        "yang sudah terbukti lewat serial; 100 membanjiri "
                        "link telemetri."),
        DeclareLaunchArgument("mission_timeout_s", default_value="180.0"),
        DeclareLaunchArgument("hover_settle_s", default_value="5.0"),
        DeclareLaunchArgument("auto_reverse", default_value="false",
                              description="false = berhenti di goal (JANGAN "
                                          "true untuk penerbangan pertama)"),

        # ── Keselamatan ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="DEFAULT TRUE. Pipeline penuh TANPA arming & TANPA "
                        "setpoint. Set false hanya setelah dry run bersih."),
        DeclareLaunchArgument("fence_fwd",  default_value="10.0"),
        DeclareLaunchArgument("fence_back", default_value="3.0"),
        DeclareLaunchArgument("fence_lat",  default_value="4.0"),
        DeclareLaunchArgument("max_home_dist", default_value="12.0",
                              description="Radius abort keras dari home (m)"),
        DeclareLaunchArgument(
            "max_alt_error", default_value="0.5",
            description="Deviasi z maksimal dari cruise_z sebelum ABORT (m). "
                        "0.5 -> jendela terbang target_alt +/- 0.5."),
        DeclareLaunchArgument("min_battery_v",   default_value="0.0",
                              description="Tegangan minimum (V). 0 = nonaktif. "
                                          "4S LiPo: ~14.4 V wajar."),
        DeclareLaunchArgument("min_battery_pct", default_value="0.0"),
        DeclareLaunchArgument("rc_override_enabled", default_value="true"),
        DeclareLaunchArgument(
            "verify_rc_override_param", default_value="true",
            description="Cek COM_RC_OVERRIDE di PX4 sebelum terbang. false "
                        "hanya untuk uji meja tanpa RC ter-bind."),
        DeclareLaunchArgument("descent_speed",    default_value="0.3"),
        DeclareLaunchArgument("land_handoff_alt", default_value="0.25"),
        DeclareLaunchArgument("auto_land_mode",   default_value="true"),
        DeclareLaunchArgument(
            "write_px4_params", default_value="false",
            description="true = node menulis MPC_XY_* / EKF2_HGT_REF. Default "
                        "false: jangan biarkan node mengubah parameter FCU "
                        "Anda diam-diam."),
        DeclareLaunchArgument("px4_vel_cap", default_value="2.0",
                              description="Dipakai hanya bila write_px4_params:=true"),
        DeclareLaunchArgument(
            "ekf_pre_wait_s", default_value="10.0",
            description="Jeda konvergensi awal GPS/EKF sebelum std-check "
                        "ground_z (sama seperti takeoff_land_node.py)."),

        # ── Guard perencana ──────────────────────────────────────────────────
        # ESCAPE MODE DIMATIKAN (permintaan operator, 2026-07-27).
        # use_safety_guards adalah SATU-SATUNYA gerbang escape: keempat titik
        # pemicu _enter_escape() di fm_inference_base.py (guard tabrakan,
        # guard look-ahead, no-progress, dan replan gagal 2x) semuanya
        # dibungkus `if self._use_guards`. Dengan false + use_lookahead_guard
        # true, guard 1 & 2 TETAP mendeteksi bahaya tetapi aksinya menjadi
        # "invalidate + hover" — drone berhenti di tempat dan replan segera,
        # bukan meluncur ke arah probe 8-arah.
        # Kalau semua replan tetap gagal: peta di-reset otomatis, lalu
        # stuck_abort_s (60 s) / blind_abort_s (10 s) yang mengambil alih ->
        # abort -> mendarat terkendali. Tidak ada penyelamatan otomatis.
        DeclareLaunchArgument(
            "use_safety_guards", default_value="false",
            description="FALSE = escape mode MATI (guard 1&2 tetap aktif, "
                        "aksinya hover+replan). Ini juga mode 'fair' yang "
                        "dipakai untuk ablasi di simulasi, jadi hasil terbang "
                        "nyata bisa dibandingkan langsung. JANGAN set true "
                        "kecuali Anda memang ingin manuver escape kembali."),
        DeclareLaunchArgument(
            "use_lookahead_guard", default_value="true",
            description="WAJIB true selama use_safety_guards false — inilah "
                        "yang menyisakan guard 1 & 2. Kalau keduanya false, "
                        "TIDAK ADA guard sama sekali."),
        DeclareLaunchArgument(
            "safe_dis", default_value="0.8",
            description="Preferensi lunak planner (m dari PUSAT drone). "
                        "SEMENTARA dinaikkan dari default kode 0.65 -> 0.8 m."),
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

        # ── Kamera & peta ────────────────────────────────────────────────────
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("depth_info_topic",
                              default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("depth_scale", default_value="0.001",
                              description="16UC1 -> meter (Gemini 2: 1 mm/unit)"),
        DeclareLaunchArgument(
            "invalid_fill_m", default_value="10.0",
            description="Nilai untuk piksel depth tidak valid. 10.0 = 'jauh/"
                        "aman' (konvensi data latih). JANGAN 0.0 — itu berarti "
                        "'obstacle menempel di lensa'."),
        DeclareLaunchArgument("hole_fill_px", default_value="5"),
        DeclareLaunchArgument("resolution", default_value="0.15",
                              description="Resolusi octomap (m)"),
        DeclareLaunchArgument("max_range",  default_value="4.0",
                              description="Jangkauan depth efektif (m)"),
        DeclareLaunchArgument("min_range",  default_value="0.5"),
        DeclareLaunchArgument("occ_min_z",  default_value="0.6"),
        DeclareLaunchArgument("occ_max_z",  default_value="2.0"),
        DeclareLaunchArgument("publish_3d_map", default_value="false"),

        # ── Gate ketinggian untuk pengolahan depth ───────────────────────────
        # Back-projection depth -> pointcloud (dan karenanya seluruh insertion
        # octomap) hanya berjalan saat drone berada di pita
        # target_alt +/- gate_alt_margin di atas tanah. Citra depth sendiri
        # (/realsense/depth/float32) TIDAK di-gate, jadi RViz tetap menampilkan
        # depth di ketinggian berapa pun.
        # PENTING: gate sengaja TERBUKA selama drone belum pernah mencapai pita
        # itu. fm_inference_real_node menunggu depth + ESDF SEBELUM ARM (langkah
        # 4, timeout 90 s); gate yang tertutup di darat akan membuat langkah itu
        # selalu gagal. Voxel tanah yang terkumpul selama jendela itu terbuang
        # sendiri karena octomap di-reset setelah takeoff settle (langkah 13).
        DeclareLaunchArgument(
            "gate_enabled", default_value="true",
            description="false = olah depth di semua ketinggian (perilaku lama)"),
        DeclareLaunchArgument(
            "gate_alt_margin", default_value="0.5",
            description="Setengah lebar pita, meter. 0.5 -> 0.7..1.7 m untuk "
                        "target_alt 1.2. Pusat pita mengikuti target_alt."),

        # ── Pemasangan kamera pada badan drone — UKUR SENDIRI! ───────────────
        # cam_x maju(+), cam_y kiri(+), cam_z atas(+) dari pusat base_link.
        # roll/pitch/yaw memutar base_link (FLU) -> frame OPTIK kamera
        # (z ke depan, x ke kanan, y ke bawah). Salah beberapa sentimeter di
        # sini menggeser SELURUH peta -> drone menghindar ke tempat yang salah.
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
            # Pusat pita gate = target_alt yang sama dengan perencana.
            "gate_enabled":    _b("gate_enabled"),
            "gate_target_alt": _f("target_alt"),
            "gate_alt_margin": _f("gate_alt_margin"),
            "gate_pose_topic": "/mavros/local_position/pose",
            # Jendela latch ground gate = jendela konvergensi EKF perencana,
            # supaya kedua node sepakat di mana "tanah" berada.
            "gate_ground_settle_s": _f("ekf_pre_wait_s"),
        }],
    )

    # Nama node WAJIB "octomap_server_unknown": fm_inference_base memanggil
    # /octomap_server_unknown/set_parameters dan /octomap_server_unknown/reset.
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
            # keselamatan hardware
            "dry_run":             _b("dry_run"),
            "fence_fwd":           _f("fence_fwd"),
            "fence_back":          _f("fence_back"),
            "fence_lat":           _f("fence_lat"),
            "max_home_dist":       _f("max_home_dist"),
            "max_alt_error":       _f("max_alt_error"),
            "min_battery_v":       _f("min_battery_v"),
            "min_battery_pct":     _f("min_battery_pct"),
            "rc_override_enabled": _b("rc_override_enabled"),
            "verify_rc_override_param": _b("verify_rc_override_param"),
            "descent_speed":       _f("descent_speed"),
            "land_handoff_alt":    _f("land_handoff_alt"),
            "auto_land_mode":      _b("auto_land_mode"),
            "write_px4_params":    _b("write_px4_params"),
            "px4_vel_cap":         _f("px4_vel_cap"),
            "ekf_pre_wait_s":      _f("ekf_pre_wait_s"),
            # perencana (diwarisi apa adanya dari pipeline simulasi)
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
        }],
    )

    return LaunchDescription(args + [
        mavros_tf, static_tf_camera, depth_bridge, depth_to_cloud,
        octomap, fm_node,
    ])
