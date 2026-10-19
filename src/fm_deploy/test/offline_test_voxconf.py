#!/usr/bin/env python3
"""
offline_test_voxconf.py — NEW (2026-10-06, VOXCONF)

Offline checks for the voxel confirmation filter of depth_to_pointcloud_node.
Does NOT start MAVROS, does NOT arm anything, does NOT spin and does NOT talk
to any other ROS process (private DDS domain, localhost only): it builds the
node, hands _on_depth() synthetic depth images rendered from a known scene and
looks at what the node would have published.

    /usr/bin/python3 src/fm_deploy/test/offline_test_voxconf.py

Part A  VoxelConfirmFilter against a slow, obviously-correct reference
Part B  the node: off = v3 output, on = scene kept / blobs held back,
        every fail-open path, parameters, status topic
"""
import json
import math
import os
import sys
import time

os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)

import numpy as np                                             # noqa: E402
import rclpy                                                   # noqa: E402
from cv_bridge import CvBridge                                 # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped    # noqa: E402
from sensor_msgs.msg import CameraInfo                         # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import fm_deploy.depth_to_pointcloud_node as dpn               # noqa: E402
from fm_deploy.depth_to_pointcloud_node import (               # noqa: E402
    DepthToPointcloudNode, VoxelConfirmFilter, _quat_to_matrix)

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print("  PASS  {0}".format(name))
    else:
        FAIL += 1
        print("  FAIL  {0}  {1}".format(name, detail))


# ══════════════════════════════════════════════════════════════════════════
# Part A — the filter class
# ══════════════════════════════════════════════════════════════════════════

class RefFilter:
    """Same rules as VoxelConfirmFilter, one voxel at a time with dicts."""

    def __init__(self, voxel, hits, window_s, keep_s):
        self.voxel, self.hits, self.window = voxel, hits, window_s
        self.keep = max(keep_s, window_s)
        self.conf, self.pend, self.frames, self.t_last = {}, {}, [], None

    def update(self, pts, t):
        if self.t_last is not None and t < self.t_last:
            self.pend, self.frames = {}, []
            self.conf = {k: min(v, t) for k, v in self.conf.items()}
        self.t_last = t
        self.frames = [x for x in self.frames if t - x <= self.window]
        blind = len(self.frames) + 1 < self.hits
        self.frames.append(t)
        inv = 1.0 / self.voxel
        keys = [tuple(int(math.floor(c * inv)) for c in p) for p in pts]
        ok = set()
        for k in set(keys):
            ct = self.conf.get(k)
            if ct is not None and t - ct > self.keep:
                del self.conf[k]
                ct = None
            if ct is not None:
                self.conf[k] = t
                ok.add(k)
                continue
            lst = [x for x in self.pend.get(k, []) if t - x <= self.window] + [t]
            if len(lst) >= self.hits:
                self.conf[k] = t
                ok.add(k)
                self.pend.pop(k, None)
            else:
                self.pend[k] = lst
        if blind:
            return np.ones(len(keys), dtype=bool)
        return np.array([k in ok for k in keys], dtype=bool)


def random_stream(rng, n_frames, dt_choices, p_empty=0.05, back_at=None):
    """Frames of points drawn from a small volume so voxels DO repeat."""
    t = 100.0
    for i in range(n_frames):
        t += float(rng.choice(dt_choices))
        if back_at is not None and i == back_at:
            t -= 50.0
        if rng.random() < p_empty:
            yield t, np.zeros((0, 3))
            continue
        static = rng.uniform([-0.6, -0.6, 0.0], [0.6, 0.6, 0.45], (int(rng.integers(5, 60)), 3))
        noise = rng.uniform([-3.0, -3.0, -1.0], [3.0, 3.0, 2.0], (int(rng.integers(0, 40)), 3))
        yield t, np.vstack([static, noise])


def part_a():
    print("\nPart A — VoxelConfirmFilter")
    rng = np.random.default_rng(1)

    cases = [
        ("hits 2 / 1.0 s / keep 60, 10 fps",   dict(voxel=0.15, hits=2, window_s=1.0, keep_s=60.0), [0.1], None),
        ("hits 2 / 1.0 s, irregular + pauses", dict(voxel=0.15, hits=2, window_s=1.0, keep_s=3.0),  [0.1, 0.1, 0.2, 0.3, 1.4], None),
        ("hits 3 / 0.5 s / keep 0.5",          dict(voxel=0.15, hits=3, window_s=0.5, keep_s=0.5),  [0.1, 0.2, 0.3], None),
        ("hits 2 / 0.35 s, voxel 0.30",        dict(voxel=0.30, hits=2, window_s=0.35, keep_s=1.0), [0.1, 0.2, 0.4], None),
        ("hits 1 (everything passes)",         dict(voxel=0.15, hits=1, window_s=1.0, keep_s=2.0),  [0.1], None),
        ("clock goes backwards mid-stream",    dict(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0),  [0.1, 0.2], 60),
    ]
    for name, kw, dts, back in cases:
        f, r = VoxelConfirmFilter(**kw), RefFilter(**kw)
        bad = 0
        n_pts = n_pass = 0
        sorted_ok = True
        for t, pts in random_stream(rng, 160, dts, back_at=back):
            m, mr = f.update(pts, t), r.update(pts, t)
            if m.dtype != bool or m.shape != (len(pts),) or not np.array_equal(m, mr):
                bad += 1
            n_pts += len(pts)
            n_pass += int(m.sum())
            ck = f._conf_keys
            if len(ck) != len(f._conf_t) or (len(ck) > 1 and not (np.diff(ck) > 0).all()):
                sorted_ok = False
        check("{0}: same mask as the reference in every frame".format(name), bad == 0,
              "{0} frames differ".format(bad))
        check("{0}: accepted set stays sorted, unique".format(name), sorted_ok)
        if kw["hits"] == 1:
            check("{0}: nothing held back".format(name), n_pass == n_pts)
        else:
            check("{0}: some points held back, some passed ({1}/{2})".format(name, n_pass, n_pts),
                  0 < n_pass < n_pts)

    # --- the grid is octomap's: index = floor(p / voxel), negatives included --------
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=60.0)
    a = np.array([[0.01, 0.01, 0.01]])
    f.update(np.zeros((0, 3)), 0.0)                       # so the next frames are not "blind"
    check("first sighting of a voxel is held back", not f.update(a, 0.1)[0])
    check("other side of 0 is another voxel (floor, not trunc)",
          not f.update(np.array([[-0.01, 0.01, 0.01]]), 0.2)[0])
    check("same voxel again within the window passes", f.update(np.array([[0.14, 0.14, 0.14]]), 0.3)[0])
    check("next voxel (0.151) is still unconfirmed", not f.update(np.array([[0.151, 0.01, 0.01]]), 0.4)[0])
    check("negative voxel confirmed by its own second hit",
          f.update(np.array([[-0.149, 0.14, 0.0]]), 0.5)[0])

    # --- one-frame blob / window / keep ---------------------------------------------
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0)
    blob = np.array([[3.5, 0.2, 1.0], [3.52, 0.21, 1.02]])
    wall = np.array([[2.0, 0.0, 1.0]])
    f.update(wall, 0.0)
    out = [f.update(wall, 0.1 * k) for k in range(1, 6)]
    check("a surface seen every frame passes from its 2nd frame on", all(o[0] for o in out))
    m = f.update(np.vstack([wall, blob]), 0.6)
    check("blob in ONE frame is held back, the wall beside it is not",
          m[0] and not m[1] and not m[2])
    for k in range(7, 18):
        f.update(wall, 0.1 * k)                            # 1.1 s without the blob
    m = f.update(np.vstack([wall, blob]), 1.75)
    check("blob again 1.15 s later (outside the window) is held back again", not m[1] and not m[2])
    m = f.update(np.vstack([wall, blob]), 1.85)
    check("blob in two frames 0.1 s apart passes", m[1] and m[2])
    for k in range(1, 40):
        f.update(wall, 1.85 + 0.1 * k)                     # 3.9 s, blob voxel not seen
    check("accepted voxel passes at once when seen again within keep_s",
          f.update(blob, 5.80)[0])
    for k in range(1, 60):
        f.update(wall, 5.80 + 0.1 * k)                     # 5.9 s > keep_s
    check("... but has to be confirmed again after keep_s", not f.update(blob, 11.75)[0])

    # --- blind frames ---------------------------------------------------------------
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0)
    m = f.update(blob, 0.0)
    check("very first frame is let through completely (blind)", m.all() and f.blind)
    m = f.update(blob + 1.0, 0.1)
    check("second frame is filtered", (not m.any()) and not f.blind)
    m = f.update(blob + 2.0, 1.5)
    check("first frame after a pause longer than the window is blind again", m.all() and f.blind)
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0)
    for k in range(30):
        f.update(np.zeros((0, 3)), 0.1 * k)                # camera running, nothing in range
    m = f.update(blob, 3.0)
    check("blob after EMPTY frames is held back (empty frames count as frames)",
          (not m.any()) and not f.blind)
    f = VoxelConfirmFilter(voxel=0.15, hits=3, window_s=1.0, keep_s=5.0)
    res = [f.update(wall, 0.6 * k) for k in range(8)]      # never 3 frames within 1 s
    check("frames too sparse for hits=3 -> every frame let through, never starved",
          all(r.all() for r in res) and f.blind)

    # --- bad input ------------------------------------------------------------------
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0)
    f.update(wall, 0.0)
    raised = 0
    for bad_pt in ([np.nan, 0, 0], [np.inf, 0, 0], [0, -np.inf, 0], [1e9, 0, 0]):
        try:
            f.update(np.array([bad_pt], dtype=float), 0.1)
        except ValueError:
            raised += 1
    check("NaN / inf / absurd coordinates raise ValueError (caller fails open)", raised == 4)
    check("... and the filter still works afterwards", f.update(wall, 0.2)[0])
    m = f.update(np.zeros((0, 3)), 0.3)
    check("empty frame -> empty bool mask", m.dtype == bool and m.shape == (0,))
    m = f.update(np.array([[2.0, 0.0, 1.0], [2.0, 0.0, 1.0]], dtype=np.float32), 0.4)
    check("float32 input, duplicate points", m.tolist() == [True, True])

    # --- far from the origin (EKF local positions of tens of metres, negative z) ----
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=5.0)
    far = np.array([[-43.40, 66.82, 9.17], [27.8, 23.7, -140.1], [-4000.0, 4000.0, 300.0]])
    f.update(np.zeros((0, 3)), 0.0)
    m1, m2 = f.update(far, 0.1), f.update(far + 0.001, 0.2)
    check("large / negative coordinates keep distinct voxels", (not m1.any()) and m2.all()
          and len(f._conf_keys) == 3)

    # --- memory cap -----------------------------------------------------------------
    f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=1000.0, max_voxels=200)
    t = 0.0
    last = None
    for k in range(40):
        pts = np.c_[np.arange(30) * 0.15 + 0.05, np.full(30, k * 0.15 + 0.05), np.zeros(30)]
        t += 0.1
        f.update(pts, t)
        t += 0.1
        f.update(pts, t)
        last = pts
    check("accepted set respects max_voxels ({0} <= 200)".format(len(f._conf_keys)),
          len(f._conf_keys) <= 200 and (np.diff(f._conf_keys) > 0).all())
    check("... and what was seen last is still accepted", f.update(last, t + 0.1).all())

    # --- cost -----------------------------------------------------------------------
    for n in (15000, 76800):
        f = VoxelConfirmFilter(voxel=0.15, hits=2, window_s=1.0, keep_s=60.0)
        ts = []
        for k in range(40):
            wall = np.c_[rng.uniform(2.9, 3.1, n // 2), rng.uniform(-2, 2, n // 2), rng.uniform(1, 3, n // 2)]
            noise = np.c_[rng.uniform(3.0, 4.0, n - n // 2), rng.uniform(-4, 4, n - n // 2),
                          rng.uniform(0, 4, n - n // 2)]
            p = np.vstack([wall, noise])
            t0 = time.perf_counter()
            f.update(p, 0.1 * k)
            ts.append((time.perf_counter() - t0) * 1e3)
        print("        {0:6d} points/frame: {1:.1f} ms median, {2:.1f} ms max".format(
            n, float(np.median(ts[5:])), max(ts[5:])))
        check("update() of {0} points stays under 40 ms".format(n), max(ts[5:]) < 40.0)


# ══════════════════════════════════════════════════════════════════════════
# Part B — the node
# ══════════════════════════════════════════════════════════════════════════

W, H = 640, 480
FX, FY, CX, CY = 304.9, 365.9, 320.0, 240.0          # Gemini 2 1280x800 resized to 640x480
Q_MOUNT = (-0.5, 0.5, -0.5, 0.5)                     # base_link <- camera optical (launch default)
T_MOUNT = np.array([0.10, 0.00, -0.05])
BRIDGE = CvBridge()


class Sink:
    """Stands in for a publisher: keeps the messages instead of sending them."""

    def __init__(self, subscribers=1):
        self.msgs, self.subscribers = [], subscribers

    def publish(self, msg):
        self.msgs.append(msg)

    def get_subscription_count(self):
        return self.subscribers


class FakeTf:
    """Stands in for tf2_ros.Buffer: answers with whatever the test put in."""

    def __init__(self):
        self.tf, self.calls, self.error = None, 0, None

    def set(self, pos, yaw, stamp_s, pitch=0.0):
        r_wb = rot_zyx(yaw, pitch)
        r_wc = r_wb @ _quat_to_matrix(*Q_MOUNT)
        t_wc = np.asarray(pos, dtype=float) + r_wb @ T_MOUNT
        q = mat_to_quat(r_wc)
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = "odom", "camera_depth_frame"
        m.header.stamp.sec = int(stamp_s)
        m.header.stamp.nanosec = int(round((stamp_s - int(stamp_s)) * 1e9)) % 1000000000
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t_wc
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        self.tf, self.r_wc, self.t_wc = m, r_wc, t_wc

    def lookup_transform_core(self, target, source, when):
        self.calls += 1
        if self.error is not None:
            raise self.error
        if self.tf is None:
            raise LookupError('"odom" passed to lookupTransform argument target_frame does not exist.')
        return self.tf


def rot_zyx(yaw, pitch=0.0):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rz @ ry


def mat_to_quat(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w > 1e-6:
        return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
                (m[1, 0] - m[0, 1]) / (4 * w), w)
    x = math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0
    return (x, (m[0, 1] + m[1, 0]) / (4 * x), (m[0, 2] + m[2, 0]) / (4 * x),
            (m[2, 1] - m[1, 2]) / (4 * x))


def render(r_wc, t_wc, rng=None, n_blobs=0, wall_x=3.0, ground=True):
    """Depth image (32FC1, m) of a static scene seen from the given camera pose:
    ground plane z = 0 (optional) and a wall x = wall_x, |y| < 1.5, z < 3. No
    return = 10 m (what the depth bridge writes). Optionally `n_blobs` 8x8 px
    patches with a random depth of 3..4 m somewhere in the picture = one-frame
    noise. Returns (depth, blob_mask)."""
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, dtype=float)], axis=-1)
    d_w = d_cam @ r_wc.T
    depth = np.full((H, W), 10.0)
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (0.0 - t_wc[2]) / d_w[..., 2]                       # ground
        ok = (s > 0) & np.isfinite(s) & ground
        depth[ok] = np.minimum(depth[ok], s[ok])
        s = (wall_x - t_wc[0]) / d_w[..., 0]                    # wall
        py = t_wc[1] + s * d_w[..., 1]
        pz = t_wc[2] + s * d_w[..., 2]
        ok = (s > 0) & np.isfinite(s) & (np.abs(py) < 1.5) & (pz > 0) & (pz < 3.0)
        depth[ok] = np.minimum(depth[ok], s[ok])
    blob = np.zeros((H, W), dtype=bool)
    for _ in range(n_blobs):
        bu, bv = int(rng.integers(0, W - 8)), int(rng.integers(0, H - 8))
        depth[bv:bv + 8, bu:bu + 8] = rng.uniform(3.0, 4.0)
        blob[bv:bv + 8, bu:bu + 8] = True
    return depth.astype(np.float32), blob


def v3_points(depth, min_range=0.5, max_range=4.0, skip=2):
    """The back-projection of depth_to_pointcloud_node v3, written out again
    here. Returns (points float32 (N,3), flat pixel index of every point)."""
    v_idx, u_idx = np.mgrid[0:H:skip, 0:W:skip]
    z = depth[v_idx, u_idx]
    valid = (z >= min_range) & (z <= max_range) & np.isfinite(z)
    z_v = z[valid].astype(np.float32)
    u_v = u_idx[valid].astype(np.float32)
    v_v = v_idx[valid].astype(np.float32)
    x = (u_v - CX) / FX * z_v
    y = (v_v - CY) / FY * z_v
    pts = np.column_stack([x, y, z_v]).astype(np.float32)
    return pts, (v_idx[valid] * W + u_idx[valid])


def make_node(**params):
    args = ["--ros-args", "--log-level", "depth_to_pointcloud_node:=error"]
    for k, v in params.items():
        args += ["-p", "{0}:={1}".format(k, v)]
    rclpy.init(args=args)
    node = DepthToPointcloudNode()
    info = CameraInfo()
    info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    node._on_info(info)
    node.pub_cloud = Sink()
    node.pub_gate = Sink()
    if node._confirm_on:
        node._tf_buffer = FakeTf()
        node._pub_rejected = Sink()
        node._pub_confirm = Sink()
    node.test_clock = [1000.0]

    class _Clock:
        def now(self_inner):
            return rclpy.time.Time(nanoseconds=int(round(node.test_clock[0] * 1e9)))
    node.get_clock = lambda: _Clock()
    return node


def kill_node(node):
    lis = getattr(node, "_tf_listener", None)
    node.destroy_node()
    rclpy.shutdown()
    if lis is not None:                      # its thread must end with the context
        lis.dedicated_listener_thread.join(timeout=5.0)
        check("TF listener thread stops at shutdown", not lis.dedicated_listener_thread.is_alive())


def feed(node, depth):
    msg = BRIDGE.cv2_to_imgmsg(depth, encoding="32FC1")
    msg.header.frame_id = "camera_depth_frame"
    n0 = len(node.pub_cloud.msgs)
    node._on_depth(msg)
    new = node.pub_cloud.msgs[n0:]
    return new[0] if new else None


def cloud_xyz(msg):
    if msg is None:
        return np.zeros((0, 3), dtype=np.float32)
    return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(-1, 3)


def kept_mask(all_pts, out_pts):
    """Which rows of all_pts were published (checks subset + order)."""
    index = {r.tobytes(): i for i, r in enumerate(all_pts)}
    idx = [index.get(r.tobytes(), -1) for r in out_pts]
    ok = (-1 not in idx) and all(b > a for a, b in zip(idx, idx[1:]))
    m = np.zeros(len(all_pts), dtype=bool)
    if ok:
        m[idx] = True
    return m, ok


def fly(node, rng, n_frames, n_blobs, pose_at, dt=0.1, tf_lag=0.02):
    """Feed n_frames rendered along pose_at(k) -> (pos, yaw). Returns per-frame
    dicts with the counts the checks need."""
    rows = []
    for k in range(n_frames):
        pos, yaw = pose_at(k)
        node.test_clock[0] += dt
        node._tf_buffer.set(pos, yaw, node.test_clock[0] - tf_lag)
        depth, blob = render(node._tf_buffer.r_wc, node._tf_buffer.t_wc, rng, n_blobs)
        pts, pix = v3_points(depth)
        out = cloud_xyz(feed(node, depth))
        m, subset = kept_mask(pts, out)
        isblob = blob.ravel()[pix]
        rej = cloud_xyz(node._pub_rejected.msgs[-1]) if node._pub_rejected.msgs else None
        rows.append(dict(k=k, n=len(pts), subset=subset, n_out=len(out),
                         scene=int((~isblob).sum()), scene_out=int((m & ~isblob).sum()),
                         blob=int(isblob.sum()), blob_out=int((m & isblob).sum()),
                         rej_ok=(rej is not None and np.array_equal(rej, pts[~m])),
                         pts=pts, mask=m))
    return rows


def part_b():
    print("\nPart B — DepthToPointcloudNode")
    rng = np.random.default_rng(7)
    still = lambda k: ((0.0, 0.0, 1.5), 0.0)                                   # noqa: E731
    cruise = lambda k: ((0.05 * k, 0.02 * k, 1.5), math.radians(2.0 * k))      # noqa: E731  0.54 m/s, 20 deg/s

    # ---- B1: filter off = v3, byte for byte ----------------------------------------
    for label, kw in (("default (no parameter)", {}), ("confirm_hits:=1", {"confirm_hits": 1}),
                      ("confirm_hits:=0", {"confirm_hits": 0})):
        node = make_node(gate_enabled="false", **kw)
        same = True
        for k in range(6):
            node.test_clock[0] += 0.1
            r_wc = rot_zyx(0.1 * k) @ _quat_to_matrix(*Q_MOUNT)
            depth, _ = render(r_wc, np.array([0.1 * k, 0.0, 1.5]), rng, n_blobs=20)
            msg = feed(node, depth)
            exp, _ = v3_points(depth)
            st = msg.header.stamp
            same &= (bytes(msg.data) == exp.tobytes() and msg.width == len(exp) and msg.height == 1
                     and msg.point_step == 12 and msg.row_step == 12 * len(exp)
                     and msg.header.frame_id == "camera_depth_frame"
                     and abs(st.sec + st.nanosec * 1e-9 - node.test_clock[0]) < 1e-6
                     and [f.name for f in msg.fields] == ["x", "y", "z"])
        check("off [{0}]: cloud identical to v3 (bytes, layout, frame, stamp)".format(label), same)
        topics = sorted(t for t, _ in node.get_topic_names_and_types())
        pubs = sorted(p.topic_name for p in node.publishers)
        subs = sorted(s.topic_name for s in node.subscriptions)
        check("off [{0}]: no TF listener, no extra topics".format(label),
              not node._confirm_on and not hasattr(node, "_tf_buffer")
              and "/tf" not in subs and "/tf_static" not in subs
              and not any("rejected" in p or "confirm" in p for p in pubs),
              "pubs={0} subs={1} topics={2}".format(pubs, subs, len(topics)))
        kill_node(node)

    # ---- B2: filter on, hovering, 3 one-frame blobs per frame ----------------------
    # (3 x 16 points/frame at a random spot 3-4 m away. Much denser noise hits
    # the same voxel twice within a second by chance and gets through — that is
    # the limit of a temporal filter, not something this test can wish away.)
    node = make_node(gate_enabled="false", confirm_hits=2)
    subs = sorted(s.topic_name for s in node.subscriptions)
    pubs = sorted(p.topic_name for p in node.publishers)
    lis = node._tf_listener
    check("on: offers points_rejected + confirm_status",
          "/realsense/depth/points_rejected" in pubs and "/realsense/depth/confirm_status" in pubs)
    check("on: TF listener has its OWN node + thread (/tf is not on this node's executor)",
          "/tf" not in subs and "/tf_static" not in subs and lis.node is not node
          and lis.dedicated_listener_thread.is_alive()
          and sorted(s.topic_name for s in lis.node.subscriptions) == ["/tf", "/tf_static"])
    rows = fly(node, rng, 25, 3, still)
    r0, rest = rows[0], rows[2:]
    check("hover: first frame goes out complete (nothing to compare it with yet)",
          r0["n_out"] == r0["n"] and r0["n"] > 5000)
    check("hover: every published point is an input point, in the same order",
          all(r["subset"] for r in rows))
    sc = sum(r["scene_out"] for r in rest) / max(1, sum(r["scene"] for r in rest))
    bl = sum(r["blob_out"] for r in rest) / max(1, sum(r["blob"] for r in rest))
    print("        hover : scene points passed {0:.2f} %, blob points passed {1:.2f} % "
          "({2} scene / {3} blob points per frame)".format(
              100 * sc, 100 * bl, rest[0]["scene"], rest[0]["blob"]))
    check("hover: static scene passes completely from frame 3 on", sc == 1.0)
    check("hover: one-frame blobs are held back (< 10 % pass)", bl < 0.10 and sum(r["blob"] for r in rest) > 800)
    check("hover: points_rejected = exactly the points held back", all(r["rej_ok"] for r in rows))
    tot = node._ctot
    check("hover: counters add up",
          tot["frames"] == 25 and tot["blind"] == 1 and tot["bypassed"] == 0
          and tot["pts_in"] == sum(r["n"] for r in rows) and tot["pts_out"] == sum(r["n_out"] for r in rows),
          str(tot))
    sts = [json.loads(m.data) for m in node._pub_confirm.msgs]
    check("hover: confirm_status is valid JSON with growing counters",
          all(s["hits"] == 2 and s["voxel_m"] == 0.15 and s["window_s"] == 2.0 for s in sts)
          and [s["frames"] for s in sts] == sorted(s["frames"] for s in sts)
          and sts[-1]["accepted_voxels"] > 100 and sts[-1]["pts_out"] <= sts[-1]["pts_in"], str(sts[-1]))
    check("hover: status is throttled to 1 Hz ({0} messages in 2.5 s)".format(len(node._pub_confirm.msgs)),
          2 <= len(node._pub_confirm.msgs) <= 4)
    check("hover: TF asked for the newest transform odom <- camera frame, once per frame",
          node._tf_buffer.calls == 25)
    kill_node(node)

    # ---- B3: filter on, flying 0.54 m/s while yawing 20 deg/s ----------------------
    node = make_node(gate_enabled="false", confirm_hits=2)
    rows = fly(node, rng, 40, 3, cruise)
    rest = rows[2:]
    sc = sum(r["scene_out"] for r in rest) / max(1, sum(r["scene"] for r in rest))
    bl = sum(r["blob_out"] for r in rest) / max(1, sum(r["blob"] for r in rest))
    worst = min(r["scene_out"] / max(1, r["scene"]) for r in rest)
    print("        cruise: scene points passed {0:.2f} % (worst frame {1:.1f} %), blob points passed "
          "{2:.2f} %".format(100 * sc, 100 * worst, 100 * bl))
    check("cruise: scene keeps passing while moving + yawing (> 95 %, worst frame > 90 %)",
          sc > 0.95 and worst > 0.90)
    check("cruise: blobs held back (< 10 % pass)", bl < 0.10)
    check("cruise: published points are an ordered subset", all(r["subset"] for r in rows))
    # the voxels the filter accepted are where the scene really is, in odom
    f = node._confirm
    k = f._conf_keys
    ijk = np.stack([(k >> 42) - f._OFF, ((k >> 21) & ((1 << 21) - 1)) - f._OFF,
                    (k & ((1 << 21) - 1)) - f._OFF], axis=1)
    c = (ijk + 0.5) * 0.15
    on_ground = np.abs(c[:, 2]) < 0.16
    on_wall = (np.abs(c[:, 0] - 3.0) < 0.16) & (np.abs(c[:, 1]) < 1.66) & (c[:, 2] > -0.16) & (c[:, 2] < 3.16)
    frac = float((on_ground | on_wall).mean())
    print("        cruise: {0} accepted voxels, {1:.1f} % on the ground plane / the wall".format(len(k), 100 * frac))
    check("cruise: accepted voxels lie on the real ground and wall (odom frame is right)",
          frac > 0.97 and on_wall.sum() > 50 and on_ground.sum() > 200)
    kill_node(node)

    # ---- B4: a wall that comes into range is delayed by one frame only -------------
    node = make_node(gate_enabled="false", confirm_hits=2)
    approach = lambda k: ((-2.2 + 0.05 * k, 0.0, 1.5), 0.0)    # noqa: E731  0.5 m/s towards the wall
    first_in = first_out = None
    for k in range(60):
        pos, yaw = approach(k)
        node.test_clock[0] += 0.1
        node._tf_buffer.set(pos, yaw, node.test_clock[0] - 0.02)
        depth, _ = render(node._tf_buffer.r_wc, node._tf_buffer.t_wc, ground=False)
        pts, _ = v3_points(depth)
        out = cloud_xyz(feed(node, depth))
        if len(pts) and first_in is None:
            first_in = k
        if len(out) and first_out is None:
            first_out = k
    check("approach: wall appears in the cloud one frame after it enters range "
          "(frame {0} -> {1})".format(first_in, first_out),
          first_in is not None and first_in > 5 and first_out == first_in + 1)
    kill_node(node)

    # ---- B5: fail-open paths --------------------------------------------------------
    node = make_node(gate_enabled="false", confirm_hits=2)
    r_wc0 = rot_zyx(0.0) @ _quat_to_matrix(*Q_MOUNT)

    def go(dt=0.1):
        """One frame: the static scene + 3 NEW blobs. -> ('complete' | 'filtered' | other)"""
        node.test_clock[0] += dt
        depth, _ = render(r_wc0, np.array([0.1, 0.0, 1.45]), rng, 3)
        exp, _ = v3_points(depth)
        out = cloud_xyz(feed(node, depth))
        if np.array_equal(out, exp):
            return "complete"
        return "filtered" if 0.9 * len(exp) < len(out) < len(exp) else "?? {0}/{1}".format(len(out), len(exp))

    def tf_now(offset=0.0):
        """TF whose stamp will be `offset` s away from the clock of the NEXT go()."""
        node._tf_buffer.set((0, 0, 1.5), 0.0, node.test_clock[0] + 0.1 + offset)

    res = [go() for _ in range(4)]                                   # no TF at all
    check("no TF: every frame published complete", res == ["complete"] * 4, str(res))
    tf_now()
    res = [go(), (tf_now(), go())[1]]
    check("TF present: one complete frame, then filtering starts (new blobs missing)",
          res == ["complete", "filtered"], str(res))
    tf_now(-1.0)                                                     # pose stream stopped 1 s ago
    res = go()
    check("TF 1.0 s old: frame published complete", res == "complete", res)
    tf_now(+5.0)                                                     # clocks disagree
    res = go()
    check("TF 5 s in the future: frame published complete", res == "complete", res)
    tf_now(-0.3)
    res = go()
    check("TF 0.3 s old (below the 0.5 s limit): filtered", res == "filtered", res)
    tf_now()
    node._tf_buffer.error = RuntimeError("tf2 exploded")
    res = go()
    check("TF lookup raises: frame published complete, node survives", res == "complete", res)
    node._tf_buffer.error = None
    tf_now()
    real_update = node._confirm.update
    node._confirm.update = lambda p, t: (_ for _ in ()).throw(ZeroDivisionError("bug"))
    res = go()
    check("filter itself raises: frame published complete, node survives", res == "complete", res)
    node._confirm.update = real_update
    tf_now()
    rot = node._tf_buffer.tf.transform.rotation
    rot.x = rot.y = rot.z = rot.w = 0.0
    res = go()
    check("TF with a zero quaternion: frame published complete", res == "complete", res)
    tf_now()
    node._tf_buffer.tf.transform.translation.x = float("nan")
    res = go()
    check("TF with NaN translation: frame published complete", res == "complete", res)
    tf_now()
    res = go()
    check("TF healthy again: filtered again", res == "filtered", res)
    check("bypassed frames are counted ({0})".format(node._ctot["bypassed"]), node._ctot["bypassed"] == 10)
    node._pub_confirm.publish = lambda m: (_ for _ in ()).throw(RuntimeError("dds down"))
    node._tf_buffer.set((0, 0, 1.5), 0.0, node.test_clock[0] + 3.0)
    res = go(3.0)                                                    # > window: unfiltered frame
    check("status publisher raises: the cloud still goes out", res == "complete", res)
    kill_node(node)

    # ---- B6: empty frames, pause ----------------------------------------------------
    node = make_node(gate_enabled="false", confirm_hits=2)
    sky = np.full((H, W), 10.0, dtype=np.float32)
    one = sky.copy()
    one[100:108, 300:308] = 3.5
    n_pub = 0
    for k in range(15):
        node.test_clock[0] += 0.1
        node._tf_buffer.set((0, 0, 6.0), 0.0, node.test_clock[0])
        n_pub += feed(node, sky) is not None
    check("empty frames: nothing published (as v3), no TF lookup needed",
          n_pub == 0 and node._tf_buffer.calls == 0)
    node.test_clock[0] += 0.1
    node._tf_buffer.set((0, 0, 6.0), 0.0, node.test_clock[0])
    check("blob after empty frames is held back -> no cloud at all", feed(node, one) is None)
    check("... and an EMPTY points_rejected was sent for the empty frames, the blob for this one",
          len(node._pub_rejected.msgs) == 16 and node._pub_rejected.msgs[0].width == 0
          and node._pub_rejected.msgs[-1].width == 16)
    node.test_clock[0] += 3.0
    node._tf_buffer.set((0, 0, 6.0), 0.0, node.test_clock[0])
    check("first frame after a 3 s pause goes out complete", feed(node, one) is not None)
    check("... and is counted as blind, not as bypassed",
          node._ctot["blind"] == 1 and node._ctot["bypassed"] == 0, str(node._ctot))
    node._pub_rejected.subscribers = 0
    n0 = len(node._pub_rejected.msgs)
    node.test_clock[0] += 0.1
    node._tf_buffer.set((0, 0, 6.0), 0.0, node.test_clock[0])
    feed(node, one)
    check("points_rejected is not built when nobody subscribes", len(node._pub_rejected.msgs) == n0)
    kill_node(node)

    # ---- B7: safety net — pose jumping around, big cloud ----------------------------
    node = make_node(gate_enabled="false", confirm_hits=2)
    rows = fly(node, rng, 12, 0, still)
    jump = lambda k: ((100.0 + 7.0 * k, 50.0 + 5.0 * k, 1.5), 0.0)   # noqa: E731  8.6 m per frame
    rows = fly(node, rng, 30, 0, jump)
    held = [r["n"] - r["n_out"] for r in rows]
    t_first_full = next((r["k"] for r in rows[1:] if r["n_out"] == r["n"]), None)
    check("jumping pose: frames are held back at first ...",
          rows[1]["n"] > 5000 and all(h > 0.9 * r["n"] for h, r in zip(held[:15], rows[:15])))
    check("... and go out complete after ~2 s (frame {0})".format(t_first_full),
          t_first_full is not None and 18 <= t_first_full <= 22 and all(h == 0 for h in held[t_first_full:]))
    rows = fly(node, rng, 6, 0, still)
    check("pose stable again: filtering resumes without a gap in the cloud",
          all(r["n_out"] > 0.9 * r["n"] for r in rows[1:]))
    check("safety-net frames are counted as bypassed ({0})".format(node._ctot["bypassed"]),
          8 <= node._ctot["bypassed"] <= 12, str(node._ctot))
    kill_node(node)

    # ---- B8: parameters -------------------------------------------------------------
    for label, kw, exp_on, exp in (
        ("confirm_hits:=2.0 (float)",       dict(confirm_hits="2.0"), True, dict(_confirm_hits=2)),
        ("confirm_hits:='3' window int 2",  dict(confirm_hits="'3'", confirm_window_s=2), True,
         dict(_confirm_hits=3, _confirm_window=2.0)),
        ("silly values are clamped",        dict(confirm_hits=99, confirm_window_s=-5, confirm_voxel=0.0,
                                                 confirm_keep_s=0.01, confirm_tf_max_age_s=100), True,
         dict(_confirm_hits=20, _confirm_window=0.1, _confirm_voxel=0.02, _confirm_keep=0.1,
              _confirm_tf_age=5.0)),
        ("not a number -> default (off)",   dict(confirm_hits="abc"), False, dict(_confirm_hits=1)),
        ("launch values",                   dict(confirm_hits=2, confirm_window_s=2.0, confirm_voxel=0.15,
                                                 confirm_frame="odom"), True,
         dict(_confirm_hits=2, _confirm_window=2.0, _confirm_voxel=0.15, _confirm_keep=60.0,
              _confirm_frame="odom", _confirm_tf_age=0.5, _confirm_log_s=10.0)),
        ("only confirm_hits:=2 (defaults)", dict(confirm_hits=2), True,
         dict(_confirm_hits=2, _confirm_window=2.0, _confirm_voxel=0.15, _confirm_keep=60.0,
              _confirm_frame="odom", _confirm_tf_age=0.5, _confirm_log_s=10.0)),
    ):
        node = make_node(**kw)
        got = {k: getattr(node, k) for k in exp}
        check("param [{0}]".format(label), node._confirm_on == exp_on and got == exp, str(got))
        if node._confirm_on:
            f = node._confirm
            check("param [{0}]: filter built with them".format(label),
                  f.hits == node._confirm_hits and f.window_s == node._confirm_window
                  and f.voxel == node._confirm_voxel and f.keep_s == node._confirm_keep)
        kill_node(node)

    # ---- B9: altitude gate still in charge ------------------------------------------
    node = make_node(confirm_hits=2, gate_target_alt=1.5, gate_alt_margin=0.5, gate_ground_settle_s=0.0)
    ps = PoseStamped()
    for z in (0.0, 0.0, 1.5):                       # ground latched, then inside the band
        ps.pose.position.z = z
        node._on_pose(ps)
    node.test_clock[0] += 0.1
    node._tf_buffer.set((0, 0, 1.5), 0.0, node.test_clock[0])
    depth, _ = render(node._tf_buffer.r_wc, node._tf_buffer.t_wc)
    a = feed(node, depth)
    ps.pose.position.z = 4.0                        # above the band -> gate closes
    node._on_pose(ps)
    calls = node._tf_buffer.calls
    node.test_clock[0] += 0.1
    b = feed(node, depth)
    check("gate closed: nothing published, the filter is not even asked",
          a is not None and b is None and node._tf_buffer.calls == calls)
    kill_node(node)

    # ---- B10: the periodic log line does not throw ----------------------------------
    node = make_node(gate_enabled="false", confirm_hits=2, confirm_log_period_s=0.5)
    lines = []
    real_logger = node.get_logger()

    class _Log:
        def info(self, m, **kw):
            lines.append(m)

        def warn(self, m, **kw):
            lines.append("WARN " + m)

        error = warn
    node.get_logger = lambda: _Log()
    fly(node, rng, 14, 5, still)
    node.get_logger = lambda: real_logger
    summary = [ln for ln in lines if ln.startswith("[CONFIRM]") and "frames" in ln]
    check("log: one [CONFIRM] summary per period, no warning ({0} lines)".format(len(summary)),
          len(summary) == 2 and not any(ln.startswith("WARN") for ln in lines),
          "\n".join(lines))
    if summary:
        print("        " + summary[-1])
    kill_node(node)


if __name__ == "__main__":
    print("module under test:", dpn.__file__)
    part_a()
    part_b()
    print("\n{0} passed, {1} failed".format(PASS, FAIL))
    sys.exit(1 if FAIL else 0)
