#!/usr/bin/env python3
# fm_inference_real_node.py — v1.0 — FM-Planner inference di DRONE NYATA
"""
fm_inference_real_node.py — deployment FM-Planner pada hardware
==========================================================================
Node ini = `FMInferenceNode` (persis yang sudah terbukti di simulasi) +
LAPISAN KESELAMATAN HARDWARE yang diambil dari `takeoff_land_node.py`
(program yang sudah pernah terbang sungguhan).

PRINSIP DESAIN — dan alasannya:

  Generator kandidat, ranking, MINCO, ESDF, guard, dan escape TIDAK DISENTUH
  sama sekali; semuanya diwarisi apa adanya dari fm_inference_base /
  fm_inference_node. Kalau logika perencanaan diubah untuk hardware, hasil
  terbang nyata tidak lagi bisa dibandingkan dengan hasil simulasi, dan
  klaim "model yang sama" di paper jadi lemah. Yang ditambahkan di sini
  HANYA hal-hal yang memang tidak ada di simulasi:

    1. RC OVERRIDE  — pilot boleh merebut kendali kapan saja. Begitu mode
       berpindah dari OFFBOARD, setpoint BERHENTI seketika (bukan setelah
       loop misi sempat memeriksa).
    2. LINK LOSS    — telemetri MAVROS putus saat terbang -> berhenti
       mengirim setpoint dan biarkan failsafe PX4 yang bekerja.
    3. GEOFENCE     — batas jarak dari titik home + batas deviasi ketinggian.
       Di Gazebo, terbang keluar arena hanya merusak data; di lapangan itu
       menabrak orang.
    4. BATERAI      — abort + mendarat sebelum tegangan jatuh.
    5. FRAME HOME   — di simulasi drone selalu lahir di (0,0) menghadap +X,
       jadi goal (goal_x, 0) benar. Di lapangan origin EKF ada di posisi &
       heading apa pun saat boot. Goal di sini didefinisikan RELATIF terhadap
       home: goal = home + R(yaw_home) . [goal_dist, 0]. Arena/geofence ikut
       dihitung dari home, bukan dari koordinat absolut.
    6. DRY RUN      — menjalankan SELURUH pipeline (kamera -> octomap -> ESDF
       -> FM -> MINCO -> marker RViz) TANPA arming dan TANPA setpoint.
       Ini cara benar untuk uji pertama: baterai terpasang, propeller
       DILEPAS, drone dipegang/di atas meja, lalu amati log
       "[FM] GATE two-sided ..." dan marker kandidat. Kalau di sini sudah
       aneh, jangan pernah dinaikkan.
    7. TAKEOFF/LANDING menahan XY HOME (bukan posisi sesaat seperti di sim).
       Menahan posisi sesaat berarti setiap drift EKF ikut jadi setpoint —
       drone perlahan "mengejar" driftnya sendiri saat naik.

URUTAN FSM:
    IDLE -> connect -> cek COM_RC_OVERRIDE -> tunggu pose -> tunggu depth+ESDF
         -> EKF stabil (ground_z) -> kunci home + hitung goal & geofence
         -> warm-up model + stream setpoint -> ARM -> OFFBOARD
         -> TAKEOFF -> settle + reset octomap -> FLYING (replan FM)
         -> LANDING (descent terkendali -> AUTO.LAND) -> DONE

CARA PAKAI:
    ros2 launch fm_deploy fm_real.launch.py \
        model_path:=$HOME/drone_ws/src/fm_deploy/model/fm/fm_planner_XXXX.onnx \
        goal_dist:=6.0 target_alt:=1.2 v_max:=0.5 dry_run:=true
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import ParamGet, ParamPull
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Empty as EmptySrv

import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from fm_inference_node import FMInferenceNode


class FMInferenceRealNode(FMInferenceNode):

    # ── Konstruksi ───────────────────────────────────────────────────────────

    def __init__(self):
        # Parent menyiapkan model, ESDF, MINCO, subscriber, dan timer setpoint.
        super().__init__()

        # ── Parameter khusus hardware ────────────────────────────────────────
        # Goal dalam FRAME HOME (lihat poin 5 di docstring). goal_x dari parent
        # diabaikan saat use_home_frame=true.
        self.declare_parameter("use_home_frame", True)
        self.declare_parameter("goal_dist",      6.0)    # m, maju dari home
        self.declare_parameter("goal_lat",       0.0)    # m, geser kiri(+)/kanan(-)
        # Geofence (frame home). Kotak arena ESDF dihitung dari sini.
        self.declare_parameter("fence_fwd",      12.0)   # m di depan home
        self.declare_parameter("fence_back",      3.0)   # m di belakang home
        self.declare_parameter("fence_lat",       4.0)   # m kiri/kanan home
        self.declare_parameter("max_home_dist",  15.0)   # m, radius abort keras
        self.declare_parameter("max_alt_error",   1.0)   # m, deviasi z -> abort
        # Baterai
        self.declare_parameter("min_battery_v",   0.0)   # V, 0 = nonaktif
        self.declare_parameter("min_battery_pct", 0.0)   # %, 0 = nonaktif
        # RC / link
        self.declare_parameter("rc_override_enabled",       True)
        self.declare_parameter("verify_rc_override_param",  True)
        # Uji tanpa terbang
        self.declare_parameter("dry_run",        False)
        # Misi
        self.declare_parameter("mission_timeout_s", 180.0)
        self.declare_parameter("hover_settle_s",      5.0)
        self.declare_parameter("descent_speed",       0.3)
        self.declare_parameter("land_handoff_alt",    0.25)
        self.declare_parameter("auto_land_mode",     True)
        # Penulisan parameter PX4. Di simulasi parent menulis EKF2_HGT_REF=1
        # (referensi tinggi = GPS). Di lapangan itu BISA SALAH: kalau Anda
        # terbang indoor dengan VIO/optical-flow, memaksa GPS sebagai referensi
        # tinggi merusak estimasi. Default: JANGAN sentuh parameter PX4.
        self.declare_parameter("write_px4_params", False)
        self.declare_parameter("ekf2_hgt_ref",        -1)   # <0 = tidak ditulis
        # Jeda sebelum mulai mengukur std EKF Z. Di simulasi EKF langsung
        # konvergen (tidak butuh ini). Di lapangan, EKF baru mulai stabil
        # beberapa detik setelah PX4 mendapat fix GPS/VIO — tanpa pre-wait,
        # jendela std bisa "kebetulan" tampak stabil padahal EKF masih
        # bergerak menuju nilai akhirnya (ground_z jadi salah, offset ikut
        # terbawa ke seluruh misi). Sama seperti takeoff_land_node.py.
        self.declare_parameter("ekf_pre_wait_s",      10.0)

        self._use_home_frame = bool(self.get_parameter("use_home_frame").value)
        self._goal_dist   = float(self.get_parameter("goal_dist").value)
        self._goal_lat    = float(self.get_parameter("goal_lat").value)
        self._fence_fwd   = float(self.get_parameter("fence_fwd").value)
        self._fence_back  = float(self.get_parameter("fence_back").value)
        self._fence_lat   = float(self.get_parameter("fence_lat").value)
        self._max_home_d  = float(self.get_parameter("max_home_dist").value)
        self._max_alt_err = float(self.get_parameter("max_alt_error").value)
        self._min_batt_v  = float(self.get_parameter("min_battery_v").value)
        self._min_batt_p  = float(self.get_parameter("min_battery_pct").value)
        self._rc_ovr_en   = bool(self.get_parameter("rc_override_enabled").value)
        self._verify_rc   = bool(self.get_parameter("verify_rc_override_param").value)
        self._dry_run     = bool(self.get_parameter("dry_run").value)
        self._mission_to  = float(self.get_parameter("mission_timeout_s").value)
        self._settle_s    = float(self.get_parameter("hover_settle_s").value)
        self._descent_v   = float(self.get_parameter("descent_speed").value)
        self._land_handoff = float(self.get_parameter("land_handoff_alt").value)
        self._auto_land   = bool(self.get_parameter("auto_land_mode").value)
        self._write_px4   = bool(self.get_parameter("write_px4_params").value)
        self._ekf2_hgt    = int(self.get_parameter("ekf2_hgt_ref").value)
        self._ekf_pre_wait = float(self.get_parameter("ekf_pre_wait_s").value)

        # ── State keselamatan ────────────────────────────────────────────────
        self._rc_override = False
        self._link_lost   = False
        self._in_offboard_mission = False
        self._stream_on   = True
        self._prev_mode   = ""
        self._prev_conn   = False
        self._have_pose   = False

        self._home_locked = False
        self._home_xy     = np.zeros(2)
        self._home_yaw    = 0.0
        self._ground_z    = 0.0
        self._hold_z      = 0.0      # z yang ditahan saat IDLE/TAKEOFF/LANDING

        self._batt_v   = None
        self._batt_pct = None

        # Watchdog terpisah dari loop misi: loop misi bisa tertahan di dalam
        # _replan (bisa ratusan ms), sementara pelanggaran geofence/baterai
        # harus terdeteksi dengan irama tetap.
        self._wd_timer = self.create_timer(0.2, self._watchdog)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  MODE: DRONE NYATA (fm_inference_real_node)")
        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"  Goal          : {self._goal_dist:.1f} m ke depan, "
            f"{self._goal_lat:+.1f} m lateral "
            + ("(frame HOME)" if self._use_home_frame else "(frame ODOM absolut)"))
        self.get_logger().info(
            f"  Geofence      : fwd {self._fence_fwd:.1f} / back "
            f"{self._fence_back:.1f} / lat +-{self._fence_lat:.1f} m "
            f"| radius abort {self._max_home_d:.1f} m")
        self.get_logger().info(
            f"  RC override   : {'AKTIF' if self._rc_ovr_en else 'NONAKTIF'}"
            f" | verifikasi param: {self._verify_rc}")
        self.get_logger().info(
            f"  Baterai       : "
            + (f"min {self._min_batt_v:.1f} V " if self._min_batt_v > 0 else "")
            + (f"min {self._min_batt_p:.0f} %" if self._min_batt_p > 0 else "")
            + ("nonaktif" if self._min_batt_v <= 0 and self._min_batt_p <= 0 else ""))
        if self._dry_run:
            self.get_logger().warn(
                "  DRY RUN       : TIDAK arming, TIDAK mengirim setpoint. "
                "Pipeline penuh tetap jalan (aman untuk uji di meja).")
        self.get_logger().info("=" * 62)

    # ── Callback: deteksi RC override & link loss ────────────────────────────

    def _cb_state(self, msg):
        super()._cb_state(msg)   # mengisi _connected / _armed / _mode
        try:
            if (self._in_offboard_mission and self._prev_conn
                    and not self._connected and not self._link_lost):
                self._link_lost = True
                self.get_logger().error(
                    "[LINK] FCU TERPUTUS saat misi aktif — setpoint dihentikan.")
            # Perpindahan mode tak terduga saat misi = pilot mengambil alih.
            # AUTO.LAND dikecualikan karena kita sendiri yang menyetelnya.
            if (self._rc_ovr_en and self._in_offboard_mission
                    and self._prev_mode == "OFFBOARD"
                    and self._mode not in ("OFFBOARD", "AUTO.LAND", "")
                    and not self._rc_override):
                self._rc_override = True
                self.get_logger().error(
                    f"[RC] MODE BERUBAH: OFFBOARD -> {self._mode}")
                self.get_logger().error(
                    "[RC] PILOT MENGAMBIL ALIH — setpoint dihentikan total.")
            self._prev_mode = self._mode
            self._prev_conn = self._connected
        except Exception:
            pass

    def _cb_sensors(self, msg):
        super()._cb_sensors(msg)   # posisi/kecepatan/yaw -> _drone_state
        try:
            import json
            d = json.loads(msg.data)
            # PENTING: jangan pakai "posisi != (0,0,0)" sebagai tanda pose siap.
            # EKF PX4 memang MULAI tepat di (0,0,0) saat inisialisasi di home,
            # jadi pemeriksaan seperti itu bisa menunggu selamanya. Yang benar:
            # apakah reader benar-benar mengirim field local_x.
            if "local_x" in d:
                self._have_pose = True
            b = d.get("battery")
            if isinstance(b, dict):
                v = b.get("voltage_V")
                if v is not None and not math.isnan(float(v)):
                    self._batt_v = float(v)
            p = d.get("battery_pct")
            if p is not None:
                self._batt_pct = float(p)
        except Exception:
            pass

    # ── Setpoint: gerbang keselamatan + hold home saat naik/turun ────────────

    def _publish_cmd(self):
        """Gerbang tunggal untuk SEMUA setpoint. Ditutup seketika saat RC
        override / link loss / dry run, tanpa menunggu loop misi menyadarinya
        (celah ~0.1 s yang di takeoff_land_node sengaja ditutup juga)."""
        if self._dry_run or self._rc_override or self._link_lost \
                or not self._stream_on:
            return
        # Saat belum/tidak dalam fase FLYING, tahan XY HOME — bukan posisi
        # sesaat. Kalau memakai posisi sesaat, drift EKF ikut menjadi perintah.
        if (self._home_locked and self._mission_state != self.STATE_FLYING):
            msg = PositionTarget()
            msg.header.stamp     = self.get_clock().now().to_msg()
            msg.header.frame_id  = "map"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = (
                PositionTarget.IGNORE_VX  | PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ  | PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE)
            msg.position.x = float(self._home_xy[0])
            msg.position.y = float(self._home_xy[1])
            msg.position.z = float(self._hold_z)
            msg.yaw        = float(self._home_yaw)
            self._pub_sp.publish(msg)
            return
        super()._publish_cmd()

    # ── Watchdog keselamatan (5 Hz) ──────────────────────────────────────────

    def _watchdog(self):
        if self._mission_state not in (self.STATE_TAKEOFF, self.STATE_FLYING):
            return
        if self._abort_reason is not None:
            return
        pos = self._drone_state.global_pos

        if self._home_locked and self._max_home_d > 0.0:
            d = float(np.linalg.norm(pos[:2] - self._home_xy))
            if d > self._max_home_d:
                self._abort_reason = (
                    f"GEOFENCE: {d:.1f} m dari home (batas {self._max_home_d:.1f} m)")
                return

        if self._cruise_z is not None and self._max_alt_err > 0.0 \
                and self._mission_state == self.STATE_FLYING:
            dz = abs(float(pos[2]) - float(self._cruise_z))
            if dz > self._max_alt_err:
                self._abort_reason = (
                    f"KETINGGIAN menyimpang {dz:.2f} m dari cruise "
                    f"(batas {self._max_alt_err:.2f} m)")
                return

        if self._min_batt_v > 0.0 and self._batt_v is not None \
                and self._batt_v < self._min_batt_v:
            self._abort_reason = (
                f"BATERAI {self._batt_v:.2f} V < {self._min_batt_v:.2f} V")
            return
        if self._min_batt_p > 0.0 and self._batt_pct is not None \
                and 0.0 <= self._batt_pct < self._min_batt_p:
            self._abort_reason = (
                f"BATERAI {self._batt_pct:.0f}% < {self._min_batt_p:.0f}%")
            return

        if self._in_offboard_mission and not self._armed:
            self._abort_reason = "DISARM tak terduga saat terbang"

    # ── Verifikasi parameter PX4 (dari takeoff_land_node) ────────────────────

    def _get_px4_param_int(self, name, timeout=5.0):
        client = self.create_client(GetParameters, "/mavros/param/get_parameters")
        if client.wait_for_service(timeout_sec=timeout):
            req = GetParameters.Request()
            req.names = [name]
            res = self._call_srv(client, req, timeout=timeout)
            if res and res.values:
                v = res.values[0]
                if v.type == ParameterType.PARAMETER_INTEGER:
                    return int(v.integer_value)
                if v.type == ParameterType.PARAMETER_DOUBLE:
                    return int(v.double_value)
        client = self.create_client(ParamGet, "/mavros/param/get")
        if client.wait_for_service(timeout_sec=2.0):
            req = ParamGet.Request()
            req.param_id = name
            res = self._call_srv(client, req, timeout=timeout)
            if res and res.success:
                return int(res.value.integer or res.value.real)
        return None

    def _check_rc_override_param(self) -> bool:
        """COM_RC_OVERRIDE harus 2 atau 3, jika tidak stik RC secara FISIK tidak
        bisa merebut OFFBOARD — deteksi di _cb_state tidak akan pernah berguna."""
        if not self._rc_ovr_en or not self._verify_rc:
            return True
        pull = self.create_client(ParamPull, "/mavros/param/pull")
        if pull.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("[SAFETY] Sinkronisasi parameter PX4...")
            self._call_srv(pull, ParamPull.Request(force_pull=False), timeout=60.0)
        value, t0 = None, time.time()
        while value is None and rclpy.ok() and time.time() - t0 < 30.0:
            value = self._get_px4_param_int("COM_RC_OVERRIDE")
            if value is None:
                time.sleep(3.0)
        if value is None:
            self.get_logger().warn(
                "[SAFETY] COM_RC_OVERRIDE tidak terbaca — lanjut, tapi "
                "VERIFIKASI MANUAL di QGroundControl.")
            return True
        if value not in (2, 3):
            self.get_logger().error("=" * 62)
            self.get_logger().error(
                f"[SAFETY] COM_RC_OVERRIDE={value} — stik RC TIDAK BISA "
                "mengambil alih OFFBOARD!")
            self.get_logger().error(
                "[SAFETY] Set ke 2 di QGroundControl lalu reboot PX4.")
            self.get_logger().error("=" * 62)
            return False
        self.get_logger().info(f"[SAFETY] COM_RC_OVERRIDE={value} — OK.")
        obl = self._get_px4_param_int("COM_OBL_ACT", timeout=3.0)
        if obl is not None:
            self.get_logger().info(
                f"[SAFETY] COM_OBL_ACT={obl} (aksi failsafe bila stream "
                "setpoint berhenti — pastikan Hold/Land/Return di QGC).")
        return True

    # ── Home frame: goal + geofence ──────────────────────────────────────────

    def _lock_home(self):
        self._home_xy  = self._drone_state.global_pos[:2].copy()
        self._home_yaw = float(self._drone_state.yaw)
        self._home_locked = True

        if self._use_home_frame:
            c, s = math.cos(self._home_yaw), math.sin(self._home_yaw)
            fwd = np.array([c, s])            # arah hidung drone saat home
            lat = np.array([-s, c])           # kiri
            self._global_target = (self._home_xy
                                   + fwd * self._goal_dist
                                   + lat * self._goal_lat)
            # Geofence didefinisikan di frame home, lalu dibungkus jadi kotak
            # sejajar sumbu (AABB) karena ESDF hanya mengenal kotak sejajar
            # sumbu. AABB adalah SUPERSET dari kotak yang diputar, jadi selalu
            # lebih longgar — pengaman kerasnya tetap max_home_dist di watchdog.
            corners = []
            for f in (-self._fence_back, self._fence_fwd):
                for l in (-self._fence_lat, self._fence_lat):
                    corners.append(self._home_xy + fwd * f + lat * l)
            corners = np.array(corners)
            self._arena_x_min = float(corners[:, 0].min())
            self._arena_x_max = float(corners[:, 0].max())
            self._arena_y_min = float(corners[:, 1].min())
            self._arena_y_max = float(corners[:, 1].max())
        else:
            self._global_target = np.array([self._goal_x, 0.0])

        self._arena_center = np.array([
            (self._arena_x_min + self._arena_x_max) / 2.0,
            (self._arena_y_min + self._arena_y_max) / 2.0])
        # Tembok virtual ESDF ikut dipindah ke kotak yang baru dihitung.
        self._esdf.arena_bounds = (self._arena_x_min, self._arena_x_max,
                                   self._arena_y_min, self._arena_y_max)

        self.get_logger().info(
            f"[REAL] home=({self._home_xy[0]:.2f},{self._home_xy[1]:.2f}) "
            f"yaw={math.degrees(self._home_yaw):.0f}deg ground_z={self._ground_z:.2f} m")
        self.get_logger().info(
            f"[REAL] goal=({self._global_target[0]:.2f},"
            f"{self._global_target[1]:.2f})  arena x[{self._arena_x_min:.1f},"
            f"{self._arena_x_max:.1f}] y[{self._arena_y_min:.1f},"
            f"{self._arena_y_max:.1f}]")

    # ── Abort keras: berhenti mengirim setpoint ──────────────────────────────

    def _hard_stop(self, reason, try_auto_land=False):
        self._stream_on = False
        self._in_offboard_mission = False
        self.get_logger().error("=" * 62)
        self.get_logger().error(f"[REAL] MISI DIHENTIKAN: {reason}")
        if try_auto_land:
            self.get_logger().error("[REAL] Menyerahkan ke AUTO.LAND.")
            try:
                self._set_mode("AUTO.LAND")
            except Exception:
                pass
        else:
            self.get_logger().error(
                "[REAL] Setpoint berhenti. Kendali ada pada pilot / failsafe PX4.")
        self.get_logger().error("=" * 62)
        self._mission_state = self.STATE_DONE

    def _aborted(self) -> bool:
        """True jika ada kondisi yang mengharuskan misi berhenti sekarang."""
        return self._rc_override or self._link_lost

    # ── Tunggu ketinggian dengan pemeriksaan hardware ────────────────────────

    def _wait_altitude_real(self, target_z, tol=0.15, timeout=30.0):
        t0, stable = time.time(), None
        while rclpy.ok() and time.time() - t0 < timeout:
            if self._aborted() or self._abort_reason is not None:
                return False
            z  = float(self._drone_state.global_pos[2])
            vz = abs(float(self._drone_state.global_vel[2]))
            if abs(z - target_z) < tol and vz < 0.2:
                stable = stable or time.time()
                if time.time() - stable >= 2.0:
                    return True
            else:
                stable = None
            time.sleep(0.1)
        return False

    # ── EKF pre-wait (dari takeoff_land_node) ────────────────────────────────

    def _wait_ekf_stable(self, tol: float = 0.08, stable_dur: float = 3.0,
                         timeout: float = 45.0) -> float:
        """Override HANYA untuk menambah pre-wait sebelum memanggil std-check
        bawaan (fm_inference_base._wait_ekf_stable — TIDAK disentuh, dipanggil
        apa adanya lewat super()). Tanpa jeda ini, EKF yang baru saja dapat
        fix GPS/VIO bisa tampak "stabil" secara std padahal masih bergerak
        menuju nilai akhirnya, sehingga ground_z terekam salah dan offset itu
        terbawa ke seluruh misi. Identik dengan takeoff_land_node.py."""
        self.get_logger().info(
            f"[REAL] Menunggu {self._ekf_pre_wait:.0f}s konvergensi awal GPS/EKF...")
        t_pre = time.time()
        while rclpy.ok() and time.time() - t_pre < self._ekf_pre_wait:
            if self._aborted() or self._abort_reason is not None:
                self.get_logger().warn("[REAL] Pre-wait EKF dibatalkan (abort).")
                break
            time.sleep(0.1)
        return super()._wait_ekf_stable(tol=tol, stable_dur=stable_dur, timeout=timeout)

    # ── Pendaratan terkendali (dari takeoff_land_node) ───────────────────────

    def _controlled_descent(self, from_z):
        self._mission_state = self.STATE_LANDING
        self._cruise_z = None
        # Mendarat DI TEMPAT: pulang ke home lewat rute yang belum tentu bebas
        # obstacle jauh lebih berisiko daripada turun di posisi sekarang.
        self._home_xy = self._drone_state.global_pos[:2].copy()
        dt     = 1.0 / self._cmd_hz
        step   = self._descent_v * dt
        target = self._ground_z + self._land_handoff
        z = from_z
        self.get_logger().info(
            f"[REAL] LANDING di ({self._home_xy[0]:.2f},{self._home_xy[1]:.2f}) "
            f"-> z={target:.2f} m")
        while rclpy.ok() and z > target:
            if self._aborted():
                return
            z = max(target, z - step)
            self._hold_z = z
            if not self._armed:
                self.get_logger().info("[REAL] Auto-disarm saat turun (land detector).")
                return
            time.sleep(dt)
        self.get_logger().info(
            f"[REAL] Descent selesai di z={self._drone_state.global_pos[2]:.2f} m")

    # ── Urutan misi ──────────────────────────────────────────────────────────

    def run_sequence(self):
        # 1. Koneksi FCU
        self.get_logger().info("[REAL] Menunggu MAVROS (/px4/state)...")
        t0 = time.time()
        while rclpy.ok() and not self._connected:
            if time.time() - t0 > self._conn_timeout:
                self.get_logger().error("[REAL] Timeout koneksi FCU.")
                return self._shutdown()
            time.sleep(0.2)
        self.get_logger().info("[REAL] FCU terhubung.")

        # 2. Verifikasi COM_RC_OVERRIDE SEBELUM apa pun yang bisa terbang
        if not self._dry_run and not self._check_rc_override_param():
            return self._shutdown()

        # 3. Pose lokal valid (GPS / VIO / optical flow)
        self.get_logger().info("[REAL] Menunggu posisi lokal (/px4/sensors)...")
        t0 = time.time()
        while rclpy.ok() and not self._have_pose:
            if time.time() - t0 > 30.0:
                self.get_logger().error(
                    "[REAL] Tidak ada posisi lokal. PX4 butuh GPS/VIO/flow "
                    "untuk OFFBOARD berbasis posisi.")
                return self._shutdown()
            time.sleep(0.2)

        # 4. Persepsi siap
        self.get_logger().info("[REAL] Menunggu depth + ESDF (octomap)...")
        t0 = time.time()
        while rclpy.ok() and (self._latest_depth is None
                              or not self._esdf.is_ready()):
            if time.time() - t0 > 90.0:
                self.get_logger().error(
                    "[REAL] Timeout depth/ESDF. Periksa: driver kamera, "
                    "gemini2_depth_bridge_node, TF odom->base_link->"
                    "camera_depth_frame, dan octomap_server.")
                return self._shutdown()
            time.sleep(0.5)
        self.get_logger().info("[REAL] Depth + ESDF siap.")

        # 5. Parameter PX4 (default: tidak menyentuh apa pun — lihat docstring)
        if self._write_px4:
            if self._ekf2_hgt >= 0:
                self._set_px4_param_int("EKF2_HGT_REF", self._ekf2_hgt)
            if self._px4_vel_cap > 0.0:
                self._set_px4_param_float("MPC_XY_VEL_MAX", self._px4_vel_cap)
                self._set_px4_param_float("MPC_XY_CRUISE",  self._v_max)
                self._set_px4_param_float("MPC_XY_VEL_ALL", self._px4_vel_cap)
                self.get_logger().info(
                    f"[REAL] PX4 speed cap -> {self._px4_vel_cap:.1f} m/s")
            time.sleep(2.0)
        else:
            self.get_logger().info(
                "[REAL] Parameter PX4 TIDAK diubah (write_px4_params:=false). "
                "Setel MPC_XY_VEL_MAX / EKF2_HGT_REF lewat QGC bila perlu.")

        # 6. ground_z dari EKF
        self._ground_z = self._wait_ekf_stable()
        if abs(self._ground_z) > 3.0:
            self.get_logger().warn(
                f"[REAL] ground_z tidak wajar ({self._ground_z:.2f} m) -> 0.0")
            self._ground_z = 0.0
        self._cruise_z = self._ground_z + self._alt
        self._hold_z   = self._ground_z

        # 7. Kunci home, hitung goal & geofence
        self._lock_home()

        # 8. Octomap band + pemanasan model
        self._configure_octomap_band(self._alt)
        self.get_logger().info("[REAL] Pemanasan model...")
        self._warm_up()

        # ── DRY RUN: pipeline penuh tanpa terbang ────────────────────────────
        if self._dry_run:
            self.get_logger().warn("=" * 62)
            self.get_logger().warn("[REAL] DRY RUN — tanpa ARM, tanpa setpoint.")
            self.get_logger().warn(
                "[REAL] Amati '[INF] Replan ok', '[FM] GATE two-sided', dan "
                "marker /planner/candidates di RViz. Ctrl-C untuk berhenti.")
            self.get_logger().warn("=" * 62)
            self._mission_state = self.STATE_FLYING
            self._reset_progress()
            last = 0.0
            while rclpy.ok():
                now = time.time()
                if now - last >= self._replan_period:
                    last = now
                    self._replan()
                time.sleep(0.05)
            return

        # 9. Warm-up stream setpoint (PX4 menolak OFFBOARD tanpa stream)
        self.get_logger().info("[REAL] Warm-up stream setpoint (~2 s)...")
        time.sleep(2.0)

        # 10. ARM
        self.get_logger().info("[REAL] ARM...")
        t0, armed = time.time(), False
        while rclpy.ok() and time.time() - t0 < self._arm_timeout:
            if self._arm(True):
                time.sleep(0.8)
                if self._armed:
                    armed = True
                    break
            time.sleep(1.5)
        if not armed:
            self.get_logger().error("[REAL] ARM gagal.")
            return self._shutdown()

        # 11. OFFBOARD
        if not self._set_mode("OFFBOARD"):
            self._arm(False)
            return self._shutdown()
        t0 = time.time()
        while rclpy.ok() and self._mode != "OFFBOARD" and time.time() - t0 < 5.0:
            self._set_mode("OFFBOARD")
            time.sleep(0.3)
        if self._mode != "OFFBOARD":
            self.get_logger().error("[REAL] PX4 tidak masuk OFFBOARD.")
            self._arm(False)
            return self._shutdown()
        self._in_offboard_mission = True

        # 12. TAKEOFF
        self._mission_state = self.STATE_TAKEOFF
        self._hold_z = self._cruise_z
        self.get_logger().info(
            f"[REAL] TAKEOFF -> z={self._cruise_z:.2f} m "
            f"(fisik ~{self._alt:.1f} m)")
        stable = self._wait_altitude_real(self._cruise_z)
        if self._aborted():
            return self._hard_stop("RC override / link loss saat takeoff")
        if self._abort_reason is not None:
            return self._land_and_finish()
        if not stable:
            self.get_logger().warn("[REAL] Takeoff belum stabil — lanjut hati-hati.")

        # 13. Settle + peta bersih
        time.sleep(self._settle_s)
        if self._octo_reset_client.wait_for_service(timeout_sec=2.0):
            self._call_srv(self._octo_reset_client, EmptySrv.Request(), timeout=5.0)
            self.get_logger().info("[REAL] Octomap di-reset (peta bersih dari cruise_z)")
            time.sleep(1.5)

        # 14. FLYING — loop replan FM
        self._mission_state = self.STATE_FLYING
        self._reset_progress()
        self.get_logger().info(
            f"[REAL] MULAI -> goal=({self._global_target[0]:.2f},"
            f"{self._global_target[1]:.2f})")

        t_mission = time.time()
        last_replan = 0.0
        while rclpy.ok():
            now = time.time()
            if self._aborted():
                return self._hard_stop("RC override / link loss saat terbang")
            if self._abort_reason is not None:
                self.get_logger().error(f"[REAL] ABORT: {self._abort_reason}")
                break
            if self._mission_to > 0.0 and now - t_mission > self._mission_to:
                self.get_logger().warn(
                    f"[REAL] Timeout misi {self._mission_to:.0f} s — mendarat.")
                break

            due  = now - last_replan >= self._replan_period
            asap = (self._replan_asap and now - last_replan >= 0.3)
            if due or asap:
                last_replan = now
                self._replan_asap = False
                self._replan()

            dist = float(np.linalg.norm(
                self._drone_state.global_pos[:2] - self._global_target))
            if not self._escape_active and (dist < 1.0 or self._reached_target):
                self.get_logger().info(f"[REAL] GOAL TERCAPAI (sisa {dist:.2f} m)")
                break
            time.sleep(0.05)

        # 15. LANDING
        self._land_and_finish()

    def _land_and_finish(self):
        if self._aborted():
            return self._hard_stop("RC override / link loss")
        with self._traj_lock:
            self._traj.invalidate()
        self._controlled_descent(float(self._drone_state.global_pos[2]))
        if self._aborted():
            return self._hard_stop("RC override / link loss saat mendarat")

        self._in_offboard_mission = False
        if self._auto_land:
            self.get_logger().info("[REAL] Touchdown diserahkan ke AUTO.LAND...")
            self._stream_on = False
            time.sleep(0.2)
            self._set_mode("AUTO.LAND")
            t0 = time.time()
            while rclpy.ok() and self._armed and time.time() - t0 < 20.0:
                time.sleep(0.3)
            if self._armed:
                self.get_logger().warn("[REAL] Land detector lambat — paksa disarm.")
                self._arm(False)
        else:
            self._arm(False)

        self._mission_state = self.STATE_DONE
        self.get_logger().info("=" * 62)
        self.get_logger().info(
            f"[REAL] SELESAI. Replan sukses: {self._replan_count}, "
            f"veto cost-gate: {self._n_cost_reject}, "
            f"veto post-check: {self._n_graze_reject}")
        if self._gate_total:
            frac = 100.0 * self._gate_two_sided / self._gate_total
            self.get_logger().info(
                f"[REAL] Bimodality gate: {self._gate_two_sided}/"
                f"{self._gate_total} replan dua-sisi ({frac:.0f}%)")
        if self._abort_reason:
            self.get_logger().error(f"[REAL] (ABORT: {self._abort_reason})")
        self.get_logger().info("=" * 62)
        self._shutdown()

    def _shutdown(self):
        self._stream_on = False
        try:
            self._cmd_timer.cancel()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FMInferenceRealNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    seq = threading.Thread(target=node.run_sequence, daemon=True)
    seq.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("[REAL] Ctrl-C — setpoint dihentikan.")
        node._stream_on = False
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
