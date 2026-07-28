#!/usr/bin/env python3
"""
fm_all.launch.py
=================
Satu terminal untuk semuanya: MAVROS + driver kamera Orbbec Gemini 2 +
seluruh stack fm_deploy (bridge, pointcloud, octomap, FM inference).
Tidak menjalankan RViz — pakai `rviz2` di terminal terpisah kalau perlu.

Padanan gabungan dari:
    T1  ros2 launch fm_deploy mavros_only.launch.py fcu_url:=...
    T2  ros2 launch orbbec_camera gemini2.launch.py
    T3  ros2 launch fm_deploy fm_real.launch.py model_path:=... dry_run:=true

Urutan start dijaga lewat TimerAction (MAVROS -> kamera -> fm stack) supaya
log tidak bertabrakan dan `_kill_stale` di mavros_only.launch.py sempat
selesai sebelum node lain naik. Ini bukan syarat keras ROS (topic/TF sudah
late-binding — node yang start lebih dulu cuma menunggu data), cuma bikin
log lebih runut dan cocok dengan urutan yang didokumentasikan.

Hanya parameter yang paling sering diubah (README §5) yang diekspos di sini.
Parameter fm_real.launch.py lain (guard perencana, resolusi octomap, dst.)
tetap memakai default aslinya — jalankan fm_real.launch.py langsung kalau
perlu menyetel itu.

Contoh:
    ros2 launch fm_deploy fm_all.launch.py \\
        fcu_url:=/dev/ttyACM0:57600 \\
        model_path:=/home/jeremy/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.pth \\
        dry_run:=true
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_FM_DEPLOY_SHARE = get_package_share_directory("fm_deploy")
_ORBBEC_SHARE = get_package_share_directory("orbbec_camera")


def generate_launch_description():
    args = [
        # ── Komunikasi ───────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "fcu_url", default_value="/dev/ttyACM0:57600",
            description="FCU URL — USB: /dev/ttyACM0:57600 | Jetson UART: /dev/ttyTHS1:921600"),

        # ── Model ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "model_path",
            description="Path checkpoint FM (.onnx atau .pth)"),
        DeclareLaunchArgument("K", default_value="8"),
        DeclareLaunchArgument(
            "onnx_fallback_on_oom", default_value="true",
            description="true = kalau .pth gagal dimuat karena CUDA "
                        "out-of-memory, otomatis pindah ke .onnx (K dipaksa "
                        "8) alih-alih node crash."),
        DeclareLaunchArgument("onnx_fallback_path", default_value=""),

        # ── Misi ─────────────────────────────────────────────────────────────
        DeclareLaunchArgument("goal_dist",  default_value="5.0"),
        DeclareLaunchArgument("goal_lat",   default_value="0.0"),
        DeclareLaunchArgument("target_alt", default_value="1.5"),
        DeclareLaunchArgument("v_max",      default_value="0.5"),
        DeclareLaunchArgument("mission_timeout_s", default_value="180.0"),

        # ── Keselamatan ──────────────────────────────────────────────────────
        DeclareLaunchArgument(
            "dry_run", default_value="true",
            description="DEFAULT TRUE. Set false hanya setelah dry run bersih."),
        DeclareLaunchArgument(
            "safe_dis", default_value="0.8",
            description="Preferensi lunak planner (m dari PUSAT drone). "
                        "Planner lebih suka jalur >= nilai ini, tapi bukan "
                        "batas keras (lihat guard_clearance/hard_clearance "
                        "untuk itu). SEMENTARA dinaikkan dari default kode "
                        "0.65 -> 0.8 m atas permintaan uji 2026-07-28."),
        DeclareLaunchArgument("fence_fwd",     default_value="10.0"),
        DeclareLaunchArgument("fence_back",    default_value="3.0"),
        DeclareLaunchArgument("fence_lat",     default_value="4.0"),
        DeclareLaunchArgument("max_home_dist", default_value="12.0"),
        DeclareLaunchArgument(
            "max_alt_error", default_value="0.5",
            description="Deviasi z maksimal dari cruise_z sebelum ABORT (m)."),
        DeclareLaunchArgument("min_battery_v", default_value="0.0"),
        DeclareLaunchArgument(
            "ekf_pre_wait_s", default_value="10.0",
            description="Jeda konvergensi awal GPS/EKF sebelum std-check "
                        "ground_z (sama seperti takeoff_land_node.py)."),

        # ── Kamera pada badan drone — UKUR SENDIRI (lihat README §4/§6) ──────
        DeclareLaunchArgument("cam_x", default_value="0.10"),
        DeclareLaunchArgument("cam_y", default_value="0.00"),
        DeclareLaunchArgument("cam_z", default_value="-0.05"),

        # ── Stream kamera — HANYA DEPTH ──────────────────────────────────────
        # Satu-satunya masukan pipeline adalah depth (/camera/depth/image_raw);
        # color dan IR tidak dipakai node mana pun. Kalau ketiganya dinyalakan
        # (default gemini2.launch.py) di port USB 2.0, color 1280x720@30 MJPG +
        # IR 1280x800@10 menghabiskan bandwidth lebih dulu dan stream DEPTH
        # GAGAL START — driver tidak memberi error, hanya diam. Akibatnya
        # octomap tidak pernah terisi, ESDF tidak pernah ready, dan
        # fm_inference_real_node berhenti di "Timeout depth/ESDF" setelah 90 s.
        # Verifikasi: depth-only -> /camera/depth/image_raw ~8 Hz.
        # Nyalakan lagi hanya untuk debug visual, dan sebaiknya di port USB 3.0.
        DeclareLaunchArgument("enable_color",       default_value="false"),
        DeclareLaunchArgument("enable_ir",          default_value="false"),
        # Point cloud dibuat depth_to_pointcloud_node, bukan driver.
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        # Tanpa color, align depth->color tidak ada gunanya. Mematikannya juga
        # membuat depth tetap di frame sensornya sendiri, konsisten dengan TF
        # base_link -> camera_depth_frame yang dipakai stack ini.
        DeclareLaunchArgument("depth_registration", default_value="false"),
        # 0 = biarkan driver memilih (terverifikasi 1280x800@10 Y16). Turunkan
        # bila latensi depth di USB 2.0 masih terlalu tinggi.
        DeclareLaunchArgument("depth_width",  default_value="0"),
        DeclareLaunchArgument("depth_height", default_value="0"),
        DeclareLaunchArgument("depth_fps",    default_value="0"),
        # KONFLIK TF — JANGAN dinyalakan. Driver Orbbec default publish_tf:=true
        # dan menerbitkan `camera_link -> camera_depth_frame` di /tf_static.
        # fm_real.launch.py menerbitkan `base_link -> camera_depth_frame` di
        # /tf_static juga. Satu frame anak dengan DUA induk: karena keduanya
        # latched, induk mana yang menang di buffer tiap subscriber bergantung
        # urutan kedatangan pesan — tidak deterministik. Kalau camera_link yang
        # menang, lookup odom->camera_depth_frame gagal, octomap membuang semua
        # cloud, /projected_map tidak pernah terbit, dan fm_inference_real_node
        # berhenti di "Timeout depth/ESDF" (gejala README §5).
        # Terverifikasi 2026-07-28: publish_tf:=false -> /tf_static kamera kosong,
        # depth tetap 9.2 Hz.
        DeclareLaunchArgument("publish_tf", default_value="false"),
        # Accel/gyro sudah default false di gemini2.launch.py, TAPI
        # enable_sync_output_accel_gyro default true tetap menerbitkan
        # /camera/accel/imu_info, /camera/gyro/imu_info dan
        # /camera/gyro_accel/sample. Tidak ada node di stack ini yang memakainya
        # — pipeline hanya butuh depth.
        DeclareLaunchArgument("enable_sync_output_accel_gyro", default_value="false"),
        DeclareLaunchArgument("enable_accel", default_value="false"),
        DeclareLaunchArgument("enable_gyro",  default_value="false"),

        # ── Gate ketinggian pengolahan depth (lihat fm_real.launch.py) ───────
        DeclareLaunchArgument("gate_enabled",    default_value="true"),
        DeclareLaunchArgument("gate_alt_margin", default_value="0.5"),
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
                    "enable_color":       LaunchConfiguration("enable_color"),
                    "enable_ir":          LaunchConfiguration("enable_ir"),
                    "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
                    "depth_registration": LaunchConfiguration("depth_registration"),
                    "depth_width":        LaunchConfiguration("depth_width"),
                    "depth_height":       LaunchConfiguration("depth_height"),
                    "depth_fps":          LaunchConfiguration("depth_fps"),
                    "publish_tf":         LaunchConfiguration("publish_tf"),
                    "enable_sync_output_accel_gyro":
                        LaunchConfiguration("enable_sync_output_accel_gyro"),
                    "enable_accel":       LaunchConfiguration("enable_accel"),
                    "enable_gyro":        LaunchConfiguration("enable_gyro"),
                }.items(),
            ),
        ],
    )

    fm_stack = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{_FM_DEPLOY_SHARE}/launch/fm_real.launch.py"),
                launch_arguments={
                    "model_path":        LaunchConfiguration("model_path"),
                    "K":                 LaunchConfiguration("K"),
                    "onnx_fallback_on_oom": LaunchConfiguration("onnx_fallback_on_oom"),
                    "onnx_fallback_path": LaunchConfiguration("onnx_fallback_path"),
                    "goal_dist":         LaunchConfiguration("goal_dist"),
                    "goal_lat":          LaunchConfiguration("goal_lat"),
                    "target_alt":        LaunchConfiguration("target_alt"),
                    "v_max":             LaunchConfiguration("v_max"),
                    "mission_timeout_s": LaunchConfiguration("mission_timeout_s"),
                    "dry_run":           LaunchConfiguration("dry_run"),
                    "safe_dis":          LaunchConfiguration("safe_dis"),
                    "fence_fwd":         LaunchConfiguration("fence_fwd"),
                    "fence_back":        LaunchConfiguration("fence_back"),
                    "fence_lat":         LaunchConfiguration("fence_lat"),
                    "max_home_dist":     LaunchConfiguration("max_home_dist"),
                    "max_alt_error":     LaunchConfiguration("max_alt_error"),
                    "min_battery_v":     LaunchConfiguration("min_battery_v"),
                    "ekf_pre_wait_s":    LaunchConfiguration("ekf_pre_wait_s"),
                    "cam_x":             LaunchConfiguration("cam_x"),
                    "cam_y":             LaunchConfiguration("cam_y"),
                    "cam_z":             LaunchConfiguration("cam_z"),
                    "gate_enabled":      LaunchConfiguration("gate_enabled"),
                    "gate_alt_margin":   LaunchConfiguration("gate_alt_margin"),
                }.items(),
            ),
        ],
    )

    return LaunchDescription(args + [mavros, camera, fm_stack])
