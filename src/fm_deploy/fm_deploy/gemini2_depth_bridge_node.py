#!/usr/bin/env python3
# gemini2_depth_bridge_node.py — v1.0 — Orbbec Gemini 2 -> kontrak input FM
"""
gemini2_depth_bridge_node.py — jembatan kamera NYATA -> kontrak input model
==========================================================================
Padanan real-world dari `gz_depth_bridge_node.py` (simulasi). Di simulasi
Gazebo mengirim /depth_camera (32FC1, meter) dan node itu meneruskannya ke
/realsense/depth/float32. Di drone nyata sumbernya adalah driver Orbbec
(`orbbec_camera`), yang menerbitkan 16UC1 dalam MILIMETER pada
/camera/depth/image_raw. Node ini menyamakan keduanya sehingga SELURUH
pipeline hilir (depth_to_pointcloud -> octomap -> ESDF, dan input model FM)
tidak perlu diubah sama sekali.

Output (identik dengan sim):
    /realsense/depth/float32       sensor_msgs/Image       32FC1, METER
    /realsense/depth/camera_info   sensor_msgs/CameraInfo  intrinsik 640x480
    /realsense/depth/z16           sensor_msgs/Image       16UC1, mm (opsional)

────────────────────────────────────────────────────────────────────────────
KENAPA NODE INI TIDAK BOLEH SEKADAR "REMAP TOPIK"
────────────────────────────────────────────────────────────────────────────
Ada tiga perbedaan fisik antara depth Gazebo dan depth Gemini 2 yang, kalau
dibiarkan, membuat model FM melihat input yang BERBEDA dari data latihnya:

1. SATUAN & TIPE. Gazebo: float32 meter. Gemini 2: uint16 milimeter.
   -> dikonversi di sini (x depth_scale, default 0.001).

2. PIKSEL TIDAK VALID (yang paling berbahaya).
   Gazebo mengembalikan +inf untuk "tidak ada obstacle sampai far clip".
   `_form_model_input` memetakan +inf -> DEPTH_NORM_MAX_M (10 m = JAUH/AMAN).
   Gemini 2 mengembalikan 0 untuk "tidak ada return" — dan 0 pada konvensi
   yang sama berarti "obstacle NEMPEL DI LENSA". Penyebab 0 di dunia nyata
   justru sering hal yang jauh/aman: permukaan mengkilap, jendela, benda di
   luar 10 m, lubang stereo. Kalau 0 diteruskan apa adanya, model akan
   melihat kabut hitam rapat di depan hidungnya dan berperilaku panik —
   padahal ruangannya kosong.
   -> parameter `invalid_fill_m` (DEFAULT 10.0 = far/aman) mengisi piksel
      tidak valid. `hole_fill_px > 0` menambal lubang KECIL dulu dengan
      median tetangga (lubang kecil di tengah tembok memang harusnya
      bernilai seperti temboknya, bukan 10 m), baru sisanya diisi far.
      SET invalid_fill_m:=0.0 HANYA jika Anda melatih ulang model dengan
      konvensi itu.

3. RESOLUSI & FOV. Model dilatih pada 640x480. Gemini 2 dapat menerbitkan
   848x480/1280x800 dsb. -> di-resize ke 640x480 dengan INTER_NEAREST
   (BUKAN interpolasi linear: rata-rata antara tepi obstacle 1 m dan latar
   8 m menghasilkan "hantu" 4.5 m yang tidak ada bendanya). Intrinsik
   camera_info ikut diskalakan supaya point cloud tetap benar secara metrik.

Selain itu depth di-clip ke [min_valid_m, DEPTH_NORM_MAX_M=10.0] karena itulah
rentang yang dipakai normalisasi input model (lihat DEPTH_NORM_MAX_M di
fm_inference_base.py / expert_planner_node.py).

────────────────────────────────────────────────────────────────────────────
CARA PAKAI
────────────────────────────────────────────────────────────────────────────
    ros2 run fm_deploy gemini2_depth_bridge_node --ros-args \
        -p input_topic:=/camera/depth/image_raw \
        -p input_info_topic:=/camera/depth/camera_info

Verifikasi sebelum terbang (drone di tangan, arahkan ke tembok ~1.5 m):
    ros2 topic hz   /realsense/depth/float32     # harus >= 10 Hz stabil
    ros2 topic echo /realsense/depth/stats       # p50 harus ~1.5, valid% tinggi
"""
import json
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

# Kontrak input model — HARUS sama dengan fm_inference_base.py
_W = 640
_H = 480
DEPTH_NORM_MAX_M = 10.0

# Fallback intrinsik kalau camera_info dari driver belum datang.
# HFOV 91 deg = Orbbec Gemini 2 (nilai yang juga dipakai di camera_vfh.launch.py).
_HFOV_FALLBACK_DEG = 91.0

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Gemini2DepthBridgeNode(Node):

    def __init__(self):
        super().__init__("gemini2_depth_bridge_node")

        self.declare_parameter("input_topic",      "/camera/depth/image_raw")
        self.declare_parameter("input_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("frame_id",         "camera_depth_frame")
        # 16UC1 mm -> meter. Gemini 2 default 1 mm/unit.
        self.declare_parameter("depth_scale",      0.001)
        # Piksel <= min_valid_m dianggap TIDAK VALID (no-return / self-view).
        self.declare_parameter("min_valid_m",      0.15)
        # Nilai pengganti piksel tidak valid. 10.0 = "jauh/aman" (lihat docstring).
        self.declare_parameter("invalid_fill_m",   DEPTH_NORM_MAX_M)
        # Ukuran kernel median untuk menambal lubang KECIL (0 = tidak menambal).
        self.declare_parameter("hole_fill_px",     5)
        self.declare_parameter("publish_z16",      False)
        self.declare_parameter("stats_period_s",   2.0)

        self._in_topic   = str(self.get_parameter("input_topic").value)
        self._info_topic = str(self.get_parameter("input_info_topic").value)
        self._frame_id   = str(self.get_parameter("frame_id").value)
        self._scale      = float(self.get_parameter("depth_scale").value)
        self._min_valid  = float(self.get_parameter("min_valid_m").value)
        self._fill       = float(self.get_parameter("invalid_fill_m").value)
        self._hole_px    = int(self.get_parameter("hole_fill_px").value)
        self._pub_z16_on = bool(self.get_parameter("publish_z16").value)
        self._stats_dt   = float(self.get_parameter("stats_period_s").value)

        if self._hole_px > 0 and self._hole_px % 2 == 0:
            self._hole_px += 1   # cv2.medianBlur butuh kernel ganjil

        self._bridge = CvBridge()
        self._src_info = None       # CameraInfo asli dari driver
        self._info_out = None       # CameraInfo hasil skala ke 640x480
        self._n_frames = 0
        self._t_stats  = self.get_clock().now()

        self._pub_f32 = self.create_publisher(
            Image, "/realsense/depth/float32", _SENSOR_QOS)
        self._pub_info = self.create_publisher(
            CameraInfo, "/realsense/depth/camera_info", _SENSOR_QOS)
        self._pub_z16 = self.create_publisher(
            Image, "/realsense/depth/z16", _SENSOR_QOS) if self._pub_z16_on else None
        self._pub_stats = self.create_publisher(
            String, "/realsense/depth/stats", 10)

        self.create_subscription(
            CameraInfo, self._info_topic, self._on_info, _SENSOR_QOS)
        self.create_subscription(
            Image, self._in_topic, self._on_depth, _SENSOR_QOS)

        self.get_logger().info("=" * 62)
        self.get_logger().info("  Gemini2 depth bridge (real hardware)")
        self.get_logger().info("=" * 62)
        self.get_logger().info(f"  in   : {self._in_topic}")
        self.get_logger().info(f"  info : {self._info_topic}")
        self.get_logger().info(f"  out  : /realsense/depth/float32  ({_W}x{_H}, 32FC1 m)")
        self.get_logger().info(
            f"  invalid px (<= {self._min_valid:.2f} m) -> {self._fill:.1f} m "
            f"| hole fill {self._hole_px} px")
        self.get_logger().info("=" * 62)

    # ── camera_info: diskalakan sekali ke 640x480 ────────────────────────────

    def _on_info(self, msg: CameraInfo):
        if self._src_info is not None and msg.width == self._src_info.width \
                and msg.height == self._src_info.height:
            return
        self._src_info = msg
        self._info_out = self._scale_info(msg)
        self.get_logger().info(
            f"[CAM] intrinsik {msg.width}x{msg.height} "
            f"fx={msg.k[0]:.1f} -> {_W}x{_H} fx={self._info_out.k[0]:.1f}")

    @staticmethod
    def _scale_info(src: CameraInfo) -> CameraInfo:
        """Intrinsik ikut diskalakan saat citra di-resize, kalau tidak point
        cloud akan salah metrik (obstacle tampak lebih lebar/sempit)."""
        sx = float(_W) / float(src.width) if src.width else 1.0
        sy = float(_H) / float(src.height) if src.height else 1.0
        out = CameraInfo()
        out.width, out.height = _W, _H
        out.distortion_model = "plumb_bob"
        out.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        k = list(src.k)
        out.k = [k[0] * sx, 0.0, k[2] * sx,
                 0.0, k[4] * sy, k[5] * sy,
                 0.0, 0.0, 1.0]
        out.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        out.p = [out.k[0], 0.0, out.k[2], 0.0,
                 0.0, out.k[4], out.k[5], 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return out

    def _fallback_info(self) -> CameraInfo:
        fx = _W / (2.0 * math.tan(math.radians(_HFOV_FALLBACK_DEG) / 2.0))
        info = CameraInfo()
        info.width, info.height = _W, _H
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, _W / 2.0, 0.0, fx, _H / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, _W / 2.0, 0.0, 0.0, fx, _H / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    # ── depth ────────────────────────────────────────────────────────────────

    def _on_depth(self, msg: Image):
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge: {exc}", throttle_duration_sec=5.0)
            return

        # 1. ke meter (float32)
        if raw.dtype == np.uint16:
            depth = raw.astype(np.float32) * self._scale
        else:
            depth = np.asarray(raw, dtype=np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        # 2. resize ke kontrak model SEBELUM menambal (lebih murah).
        #    INTER_NEAREST: interpolasi linear akan menciptakan kedalaman
        #    "antara" tepi obstacle dan latar — obstacle hantu.
        if depth.shape != (_H, _W):
            depth = cv2.resize(depth, (_W, _H), interpolation=cv2.INTER_NEAREST)

        invalid = depth <= self._min_valid
        n_invalid = int(invalid.sum())

        # 3. tambal lubang KECIL dengan median tetangga (nilai tembok, bukan far)
        if self._hole_px > 0 and n_invalid:
            filled = cv2.medianBlur(depth, self._hole_px)
            small = invalid & (filled > self._min_valid)
            depth[small] = filled[small]
            invalid = depth <= self._min_valid

        # 4. sisa piksel tidak valid -> far/aman (lihat docstring, poin 2)
        depth[invalid] = self._fill

        # 5. clip ke rentang normalisasi model
        np.clip(depth, 0.0, DEPTH_NORM_MAX_M, out=depth)

        stamp = self.get_clock().now().to_msg()
        out = self._bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        out.header.stamp    = stamp
        out.header.frame_id = self._frame_id
        self._pub_f32.publish(out)

        info = self._info_out if self._info_out is not None else self._fallback_info()
        info.header.stamp    = stamp
        info.header.frame_id = self._frame_id
        self._pub_info.publish(info)

        if self._pub_z16 is not None:
            z16 = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
            m16 = self._bridge.cv2_to_imgmsg(z16, encoding="16UC1")
            m16.header.stamp    = stamp
            m16.header.frame_id = self._frame_id
            self._pub_z16.publish(m16)

        self._n_frames += 1
        self._maybe_stats(depth, n_invalid)

    def _maybe_stats(self, depth, n_invalid):
        now = self.get_clock().now()
        dt = (now - self._t_stats).nanoseconds * 1e-9
        if dt < self._stats_dt:
            return
        self._t_stats = now
        valid = depth[(depth > self._min_valid) & (depth < DEPTH_NORM_MAX_M)]
        pct_valid = 100.0 * valid.size / float(depth.size)
        payload = {
            "hz":         round(self._n_frames / dt, 1),
            "valid_pct":  round(pct_valid, 1),
            "invalid_px": n_invalid,
            "min_m":      round(float(valid.min()), 2) if valid.size else None,
            "p50_m":      round(float(np.median(valid)), 2) if valid.size else None,
        }
        self._n_frames = 0
        self._pub_stats.publish(String(data=json.dumps(payload)))
        # Peringatan dini: kamera yang "buta" (hampir semua piksel invalid) akan
        # tetap menghasilkan citra 10 m yang tampak aman -> drone terbang membabi
        # buta. Lebih baik ribut di terminal.
        if pct_valid < 20.0:
            self.get_logger().warn(
                f"[CAM] hanya {pct_valid:.0f}% piksel valid — periksa exposure/"
                "permukaan/jarak. Depth dianggap 'jauh' di sisanya!",
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = Gemini2DepthBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
