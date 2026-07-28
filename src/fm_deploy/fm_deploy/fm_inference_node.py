#!/usr/bin/env python3
# fm_inference_node.py — v2.1 — FM-Planner deployment (trajectory-remainder anchor + bimodality gate)
"""
fm_inference_node.py — FM-Planner deployment (multimodal warm start)
==========================================================================
CHANGELOG
  v2.1 (2026-07-06)  ANCHOR REDESIGNED after the failed single-trial test
      (escapes 5 -> 10, fm_anchor_frontbox_wide_test_20260706_171129).
      Root cause confirmed: the old anchor reused ABSOLUTE world-frame
      waypoints stored 0.5 s earlier — geometrically stale once the drone
      advanced (near/behind the new head -> degenerate seed that the
      efficiency ranking could even score as artificially cheap).
      New design: the anchor is rebuilt EVERY replan from the still-ahead
      REMAINDER of the ACTIVE trajectory (sampled at head-time + 1/3 and
      2/3 of remaining duration, ts = remaining base-time thirds). It is
      by construction "the same trajectory, continued", never stale.
      Staleness guards: traj must be valid, remaining time must fit
      3 x T_min, w1 must be >= ANCHOR_MIN_AHEAD in front of the head, and
      the anchor is suppressed for one replan after an escape.
      use_anchor_sampling still defaults FALSE — validate on >= 5 trials
      (wide_test single-pass) before switching the default.
  v2.1 also adds the BIMODALITY GATE telemetry: cumulative fraction of
      replans whose candidate pool contains BOTH sides
      (|lat| > FM_BIMODAL_SIDE_LAT each way). Logged every
      GATE_LOG_EVERY successful rankings as "[FM] GATE ...". This is the
      go/no-go metric for wall_gauntlet: per command.txt the checkpoint
      must be genuinely bimodal (target >= ~30% two-sided in ambiguous
      worlds; checkpoint 20260706_130532 measured ~3% = FAIL).
  v2.0 (2026-07-06)  Anchored sampling + efficiency ranking (both OFF by
      default after the failed trial; see v2.1 note).
  v1.x (2026-07-05)  Mode persistence, bimodal early-steering, ablation
      toggles (FM-minimal vs FM+).
==========================================================================
Same pipeline as fm_inference_base (guards, escape, mission sequence are
inherited UNCHANGED — this keeps NN-vs-FM comparisons apple-to-apple).
Only the initializer differs:

  NN : 1 forward pass  -> 1 warm-start guess          -> warm_start_plan
  FM : encoder 1x + Euler n_steps -> K candidates
       -> ranked by coarse ESDF clearance (best first)
       -> warm_start_plan tries candidates until one passes the hard
          post-check (_validate_planned_traj)

Model file: checkpoint from fm_trainer.py
  .pth  : {state_dict, norm_mean, norm_std} loaded via fm_model.load_checkpoint
  .onnx : inputs 'input' [1, 640*480+24] + 'noise' [K, 9] -> 'candidates' [K, 9]
          (the Euler chain is unrolled at export time, so n_steps is fixed
          inside the graph; the n_steps parameter only affects .pth)

Extra parameters vs fm_inference_base:
  K       (default 8) : candidates sampled per replan
  n_steps (default 2) : Euler steps (.pth backend only)
  onnx_fallback_on_oom (default True) : kalau .pth gagal dimuat karena CUDA
      out-of-memory, otomatis pindah ke backend .onnx sekali, bukan crash.
      Terlihat di lapangan 2026-07-28 di Jetson Orin Nano: `torch.load(...)`
      gagal dengan "CUDA error: out of memory" walau RAM tampak cukup di
      `free -h` — root cause-nya fragmentasi memori unified Jetson
      (tegrastats melaporkan `lfb` / largest-free-block cuma puluhan MB).
      ONNX-nya sendiri sudah terverifikasi jalan di CPU di mesin yang sama,
      jadi fallback ini aman: kalau CUDAExecutionProvider juga tidak
      tersedia untuk onnxruntime (kasus di mesin ini), sesi ONNX otomatis
      turun ke CPUExecutionProvider — tidak pernah crash dua kali karena
      alasan GPU yang sama.
      K DIPAKSA ke 8 saat fallback terjadi (lihat docstring gemini2_depth_
      bridge_node.py yang sama soal keterbatasan ini: export .onnx yang ada
      mengunci noise input pada bentuk statis [8, 9]).
  onnx_fallback_path (default "") : path .onnx eksplisit dipakai saat
      fallback. Kosong = derive otomatis dari model_path (ganti ekstensi
      .pth -> .onnx, direktori sama).

Usage:
  ros2 launch fm_planner fm_planning_unknown.launch.py \
      model_path:=/home/michael/saved_net/fm/run_XXXX/fm_planner_XXXX.pth \
      goal_x:=20.0 K:=8 n_steps:=2
"""
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
import torch
from rclpy.executors import MultiThreadedExecutor

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _THIS_DIR)
from fm_model import PARAM_DIM, load_checkpoint
from fm_inference_base import (
    FMInferenceBase, HAS_ONNX, IMG_WIDTH, IMG_HEIGHT, MOTION_INPUT_SIZE,
)

if HAS_ONNX:
    import onnxruntime as ort

# Mode persistence (hysteresis) for candidate ranking:
#   MODE_COMMIT_MIN_LAT  : the drone counts as "committed to a side" once the
#                          active trajectory is laterally offset by this much (m)
#   MODE_PERSISTENCE_BONUS: clearance-equivalent bonus (m) for candidates on
#                          the committed side — the other side must be at least
#                          this much SAFER to justify switching modes mid-approach
MODE_COMMIT_MIN_LAT    = 0.2
MODE_PERSISTENCE_BONUS = 0.3

# Bimodal early-steering: candidate spread as an early-warning signal.
# When the K samples split left/right (wide obstacle fills the camera), that
# information is available the moment the DEPTH IMAGE sees the obstacle —
# hundreds of ms before the octomap/ESDF knows. Instead of letting the local
# target sit straight ahead until the map catches up (which is what drove the
# drone head-on to ~1 m before guards fired), shift the local target toward
# the best candidate's side immediately.
#   FM_BIMODAL_SIDE_LAT   : |lateral| (m) for a candidate to count as left/right
#   FM_BIMODAL_MIN_SPREAD : min gap (m) between extremes to call it bimodal
#   FM_TARGET_BIAS_MAX    : max lateral shift (m) applied to the local target
#   FM_BIAS_DECAY         : bias decay per replan once the split disappears
FM_BIMODAL_SIDE_LAT   = 0.15   # 0.3 -> 0.15: 2026-07-05 model's sample spread
FM_BIMODAL_MIN_SPREAD = 0.5    # 1.0 -> 0.5   turned out narrow (spread_y val ~0.13m,
FM_TARGET_BIAS_MAX    = 1.5    #               different-side scenes are only 5% of the dataset)
FM_BIAS_DECAY         = 0.5

# Anchored sampling v2.1 — trajectory-remainder anchor.
# Purpose: give the K-candidate pool one concrete "continue what we are
# already flying" option every replan, so mode decisions cannot flip purely
# because fresh noise failed to reproduce the previous winner.
# Design (fixes the documented v2.0 failure — stale world-frame waypoints):
#   the anchor is SAMPLED from the ACTIVE trajectory at
#       t_head + 1/3 and + 2/3 of the remaining (wall-clock) duration,
#   with ts = remaining BASE-time thirds (wall time / time_scale), clipped
#   to [T_min, T_max]. It is therefore always expressed relative to the NEW
#   head — never behind it, never degenerate.
# Guards:
#   ANCHOR_MIN_REMAIN_S : skip if remaining wall time is below this (the
#                         tail is too close for a meaningful 3-piece seed)
#   ANCHOR_MIN_AHEAD    : skip if w1 ends up closer than this to the head
#   escape              : suppressed for one replan after _enter_escape
#                         (continuity already broke — resample fresh)
# Still defaults OFF: re-validate on >= 5 wide_test single-pass trials
# before flipping the default (v2.0 lesson).
ANCHOR_MIN_REMAIN_S = 2.0   # s, wall-clock remaining below this -> no anchor
ANCHOR_MIN_AHEAD    = 0.3   # m, w1 must be at least this far ahead of head

RANK_LEN_PENALTY = 0.05   # score penalty per meter of extra polyline length
                          # (head->w1->w2->tail) — small vs clearance deltas
                          # (~0.3-2m), meant to only tie-break between
                          # similarly-safe candidates. Safe to combine with
                          # the v2.1 anchor (which can no longer be stale).

# Bimodality gate telemetry (always on, negligible cost): the go/no-go
# metric before running the wall_gauntlet discriminator world. A replan
# counts as "two-sided" when the candidate pool contains at least one
# candidate on EACH side (|lat| > FM_BIMODAL_SIDE_LAT). Cumulative fraction
# is logged every GATE_LOG_EVERY rankings and once more at mission end.
GATE_LOG_EVERY = 20


class FMInferenceNode(FMInferenceBase):

    # ── Model backend ─────────────────────────────────────────────────────────

    def _load_model(self):
        # Declared here rather than __init__: _load_model runs inside the
        # parent __init__, the earliest point where parameters can be declared.
        self.declare_parameter("K", 8)
        self.declare_parameter("n_steps", 2)
        self.declare_parameter("onnx_fallback_on_oom", True)
        self.declare_parameter("onnx_fallback_path", "")
        self._onnx_fallback      = bool(
            self.get_parameter("onnx_fallback_on_oom").value)
        self._onnx_fallback_path = str(
            self.get_parameter("onnx_fallback_path").value)
        # Ablation toggles (default OFF = "FM-minimal": nothing beyond the base
        # pipeline + the candidate generator, i.e. the historical "model swap
        # only" comparison configuration. ON = the anticipatory FM+ variant.)
        self.declare_parameter("use_mode_persistence", False)
        self.declare_parameter("use_bimodal_steering", False)
        # Sampler-level fixes (Tahap 2). v2.1 anchor = trajectory-remainder
        # design (see constants block) — the v2.0 stale-waypoint failure mode
        # is structurally gone, but keep the default OFF until re-validated
        # on >= 5 wide_test single-pass trials.
        self.declare_parameter("use_anchor_sampling", False)
        self.declare_parameter("use_efficiency_ranking", False)
        # v2.2 (2026-07-09): optional per-replan candidate dump for
        # visualization -- ALL K raw candidates (not just the one flown),
        # so "does FM genuinely consider both sides" can be checked visually
        # instead of trusting the GATE percentage alone. '' = disabled
        # (default; zero overhead). One JSON line per replan.
        self.declare_parameter("candidate_log_path", "")
        self._cand_log_path = str(
            self.get_parameter("candidate_log_path").value)
        self._cand_log_f = None
        if self._cand_log_path:
            os.makedirs(os.path.dirname(self._cand_log_path), exist_ok=True)
            self._cand_log_f = open(self._cand_log_path, "w")
        self._K       = int(self.get_parameter("K").value)
        self._n_steps = int(self.get_parameter("n_steps").value)
        self._use_persistence = bool(
            self.get_parameter("use_mode_persistence").value)
        self._use_steering = bool(
            self.get_parameter("use_bimodal_steering").value)
        self._use_anchor  = bool(self.get_parameter("use_anchor_sampling").value)
        self._use_eff_rank = bool(
            self.get_parameter("use_efficiency_ranking").value)

        self._steer_bias = 0.0   # bimodal early-steering state (see constants)
        self._anchor_hold = False  # True for 1 replan after an escape:
                                   # continuity broke, do not anchor to it
        # Bimodality gate counters (see constants block)
        self._gate_total     = 0
        self._gate_two_sided = 0

        fell_back = False
        if self._use_pth:
            try:
                self._fm = load_checkpoint(self._model_path, device=self._device)
                self._fm.eval()
                return (f"FM PyTorch ({self._device}) "
                        f"K={self._K} n_steps={self._n_steps}")
            except RuntimeError as exc:
                is_oom = "out of memory" in str(exc).lower()
                if not (is_oom and self._onnx_fallback):
                    raise
                self.get_logger().error(
                    f"[FM] CUDA OOM saat memuat .pth: {exc} — "
                    f"fallback ke .onnx (onnx_fallback_on_oom aktif)")
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
                onnx_path = (self._onnx_fallback_path
                             or os.path.splitext(self._model_path)[0] + ".onnx")
                if not os.path.isfile(onnx_path):
                    self.get_logger().error(
                        f"[FM] Fallback GAGAL: {onnx_path} tidak ditemukan. "
                        "Set onnx_fallback_path kalau nama file berbeda.")
                    raise
                if self._K != 8:
                    self.get_logger().warn(
                        f"[FM] Backend ONNX mengunci K=8 (noise input statis "
                        f"[8,9]); K={self._K} -> dipaksa 8.")
                    self._K = 8
                self._use_pth   = False
                self._model_path = onnx_path
                fell_back = True

        if not HAS_ONNX:
            raise RuntimeError("onnxruntime not installed")
        self._session = ort.InferenceSession(
            self._model_path,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
        tag = " [FALLBACK dari .pth krn CUDA OOM]" if fell_back else ""
        return f"FM ONNX ({self._session.get_providers()}) K={self._K}{tag}"

    def _sample_params(self, input_flat, k=None):
        """[k, 9] raw candidates (body frame, denormalized). k defaults to K;
        callers pass k=K-1 when 1 slot is reserved for the anchor candidate."""
        k = self._K if k is None else k
        if self._use_pth:
            with torch.no_grad():
                x = torch.tensor(input_flat).unsqueeze(0).to(self._device)
                out = self._fm.sample(x, K=k, n_steps=self._n_steps)
            return out.cpu().numpy()
        noise = np.random.randn(k, PARAM_DIM).astype(np.float32)
        return self._session.run(
            None, {'input': input_flat.reshape(1, -1), 'noise': noise})[0]

    def _anchor_from_traj(self, drone_state):
        """v2.1 anchor: the still-ahead REMAINDER of the ACTIVE trajectory,
        re-expressed as a raw [9] candidate (w1_body_xyz, w2_body_xyz,
        ts1..3) — the same convention as a fresh FM sample, so it flows
        through _initial_guesses / ranking / warm_start_plan unchanged.

        w1/w2 are sampled at t_head + 1/3 and + 2/3 of the remaining
        wall-clock duration (t_head = now + planning_time_ahead, matching
        _get_head_state); ts are the remaining BASE-time thirds
        (wall / time_scale), clipped to [T_min, T_max]. By construction the
        seed is "the same trajectory, continued" from the NEW head — the
        v2.0 staleness failure (waypoints near/behind the head) cannot
        occur. Returns None when there is no healthy trajectory, the
        remainder is too short, w1 is not meaningfully ahead, or an escape
        just happened (self._anchor_hold)."""
        if self._anchor_hold:
            self._anchor_hold = False      # suppress exactly one replan
            return None
        t_now = time.time()
        t_head = t_now + self._dt_ahead
        with self._traj_lock:
            if not self._traj.is_valid():
                return None
            remain = self._traj.time_remaining(t_head)
            if remain < max(ANCHOR_MIN_REMAIN_S,
                            3.0 * self._planner.T_min + 0.2):
                return None
            head_p, _, _ = self._traj.sample_at(t_head)
            w1, _, _ = self._traj.sample_at(t_head + remain / 3.0)
            w2, _, _ = self._traj.sample_at(t_head + 2.0 * remain / 3.0)
            scale = self._traj.time_scale if self._traj.time_scale > 1e-6 else 1.0
        if head_p is None or w1 is None or w2 is None:
            return None
        if float(np.linalg.norm(w1 - head_p)) < ANCHOR_MIN_AHEAD:
            return None                    # tail too close / near-stationary
        ts_piece = float(np.clip((remain / scale) / 3.0,
                                 self._planner.T_min + 1e-3,
                                 self._planner.T_max - 1e-3))
        quat, dpos = drone_state.attitude, drone_state.global_pos[:2]
        w1_body = quat.inverse.rotate(
            np.array([w1[0] - dpos[0], w1[1] - dpos[1], 0.0]))
        w2_body = quat.inverse.rotate(
            np.array([w2[0] - dpos[0], w2[1] - dpos[1], 0.0]))
        return np.concatenate(
            [w1_body, w2_body, np.array([ts_piece, ts_piece, ts_piece])])

    # ── K-candidate initial guesses ───────────────────────────────────────────

    def _initial_guesses(self, depth_img, drone_state, head_state, tail_state):
        depth_u8, motion_info = self._form_model_input(
            depth_img, drone_state, head_state[0], head_state[1])
        input_flat = np.concatenate(
            [depth_u8.reshape(-1).astype(np.float32), motion_info])
        t0 = time.time()
        anchor = self._anchor_from_traj(drone_state) if self._use_anchor else None
        n_fresh = self._K - 1 if anchor is not None else self._K
        params = self._sample_params(input_flat, k=n_fresh)   # [n_fresh, 9]
        if anchor is not None:
            params = np.vstack([params, anchor[None, :]])      # [K, 9]
        self._inference_times.append((time.time() - t0) * 1000)
        if len(self._inference_times) > 100:
            self._inference_times.pop(0)

        quat, dpos = drone_state.attitude, drone_state.global_pos
        guesses = []
        for out in params:
            w1 = quat.rotate(np.array(out[0:3])) + dpos
            w2 = quat.rotate(np.array(out[3:6])) + dpos
            int_wpts = np.array([[w1[0], w2[0]], [w1[1], w2[1]]])
            ts = np.clip(out[6:9], self._planner.T_min + 1e-3,
                         self._planner.T_max - 1e-3)
            guesses.append((int_wpts, ts))
        return self._rank_by_clearance(guesses, head_state[0], tail_state[0])

    def _current_side(self, head_xy, tail_xy):
        """Signed lateral offset (m) of the ACTIVE trajectory 0.8-1.6s ahead
        relative to the head->tail axis. 0.0 = no valid trajectory / straight.
        Used as the mode-persistence reference: which side is the drone
        already committed to right now."""
        axis = tail_xy - head_xy
        n = float(np.linalg.norm(axis))
        if n < 1e-3:
            return 0.0
        axis = axis / n
        t_now = time.time()
        lats = []
        with self._traj_lock:
            if not self._traj.is_valid():
                return 0.0
            for dt in (0.8, 1.6):
                p, _, _ = self._traj.sample_at(t_now + dt)
                if p is None:
                    break
                lats.append(axis[0] * (p[1] - head_xy[1])
                            - axis[1] * (p[0] - head_xy[0]))
        return float(np.mean(lats)) if lats else 0.0

    def _rank_by_clearance(self, guesses, head_xy, tail_xy):
        """Best candidate first: score = min ESDF over the coarse polyline
        head -> w1 -> w2 -> tail (waypoints + segment midpoints) + a
        mode-persistence bonus for candidates on the SAME side as the active
        trajectory. Without the bonus, K fresh samples per replan can flip
        left/right on near-symmetric scenes -> oscillation between replans;
        the bonus makes FM switch modes only when the other side is clearly
        better (hysteresis), not because of sampling noise."""
        if not self._esdf.is_ready():
            return guesses
        side = (self._current_side(head_xy, tail_xy)
                if self._use_persistence else 0.0)
        axis = tail_xy - head_xy
        n = float(np.linalg.norm(axis))
        axis = axis / n if n > 1e-3 else np.array([1.0, 0.0])
        scored = []
        for int_wpts, ts in guesses:
            w1, w2 = int_wpts[:, 0], int_wpts[:, 1]
            pts = (w1, w2,
                   0.5 * (head_xy + w1), 0.5 * (w1 + w2), 0.5 * (w2 + tail_xy))
            score = min(float(self._esdf.get_edt_dis(p)) for p in pts)
            if self._use_eff_rank:
                poly_len = (float(np.linalg.norm(w1 - head_xy))
                            + float(np.linalg.norm(w2 - w1))
                            + float(np.linalg.norm(tail_xy - w2)))
                score -= RANK_LEN_PENALTY * poly_len
            mid = 0.5 * (w1 + w2)
            cand_lat = (axis[0] * (mid[1] - head_xy[1])
                        - axis[1] * (mid[0] - head_xy[0]))
            if abs(side) > MODE_COMMIT_MIN_LAT and cand_lat * side > 0:
                score += MODE_PERSISTENCE_BONUS
            scored.append((score, cand_lat, int_wpts, ts))
        scored.sort(key=lambda s: s[0], reverse=True)
        # ── Bimodality gate telemetry (go/no-go for wall_gauntlet) ──────────
        lats_all = [s[1] for s in scored]
        self._gate_total += 1
        if (any(l >  FM_BIMODAL_SIDE_LAT for l in lats_all)
                and any(l < -FM_BIMODAL_SIDE_LAT for l in lats_all)):
            self._gate_two_sided += 1
        if self._gate_total % GATE_LOG_EVERY == 0:
            frac = self._gate_two_sided / max(self._gate_total, 1)
            self.get_logger().info(
                f"[FM] GATE two-sided {self._gate_two_sided}/{self._gate_total}"
                f" ({100.0 * frac:.0f}%) — wall_gauntlet ready if genuinely"
                f" bimodal (~>=30% in ambiguous worlds; 3% = unimodal FAIL)")
        if self._use_steering:
            self._update_steer_bias(scored)
        if self._cand_log_f is not None:
            self._cand_log_f.write(json.dumps({
                "t": time.time(),
                "replan": self._gate_total,
                "head": [float(head_xy[0]), float(head_xy[1])],
                "tail": [float(tail_xy[0]), float(tail_xy[1])],
                # rank 0 = what the ranking picked as best (tried first by
                # warm_start_plan) -- almost always what gets flown, unless
                # it fails the hard post-check and a lower rank is used
                # instead (see "cand#" in the [INF] Replan ok log line).
                "candidates": [
                    {"w1": [float(w[0, 0]), float(w[1, 0])],
                     "w2": [float(w[0, 1]), float(w[1, 1])],
                     "lat": float(lat), "score": float(sc)}
                    for sc, lat, w, t in scored
                ],
            }) + "\n")
            self._cand_log_f.flush()
        return [(w, t) for _, _, w, t in scored]

    def _update_steer_bias(self, scored):
        """Early warning from the candidate spread. `scored` = [(score, lat, w, t)]
        sorted best-first. Candidates split left+right with a large spread
        = a wide obstacle filling the camera -> set the target bias toward
        the best candidate's side RIGHT NOW (without waiting for the octomap).
        Split disappears -> bias decays."""
        lats = [s[1] for s in scored]
        if not lats:
            return
        spread = max(lats) - min(lats)
        has_left  = any(l >  FM_BIMODAL_SIDE_LAT for l in lats)
        has_right = any(l < -FM_BIMODAL_SIDE_LAT for l in lats)
        # Threshold-tuning diagnostic: candidate lateral spread every ~2s
        self.get_logger().info(
            f"[FM] lat spread={spread:.2f}m "
            f"[{min(lats):+.2f}..{max(lats):+.2f}] L={has_left} R={has_right}",
            throttle_duration_sec=2.0)
        if has_left and has_right and spread >= FM_BIMODAL_MIN_SPREAD:
            best_lat = scored[0][1]
            sign = 1.0 if best_lat > 0 else -1.0
            self._steer_bias = sign * min(FM_TARGET_BIAS_MAX, 0.5 * spread)
            self.get_logger().info(
                f"[FM] Bimodal spread={spread:.1f}m -> steer "
                f"{'LEFT' if sign > 0 else 'RIGHT'} bias={self._steer_bias:+.2f}m",
                throttle_duration_sec=1.0)
        else:
            self._steer_bias *= FM_BIAS_DECAY
            if abs(self._steer_bias) < 0.05:
                self._steer_bias = 0.0

    def _set_local_target(self, shorter=False, ref_pos=None):
        """Parent local target + shift laterally by the steer bias (if any and
        the shifted point is collision-free). This is the channel that carries
        FM's anticipatory information to the planner — not just the warm start."""
        super()._set_local_target(shorter=shorter, ref_pos=ref_pos)
        bias = self._steer_bias
        if abs(bias) < 0.05 or not self._esdf.is_ready():
            return
        curr = self._drone_state.global_pos[:2] if ref_pos is None else ref_pos
        to_goal = self._global_target - curr
        n = float(np.linalg.norm(to_goal))
        if n < 1e-3:
            return
        fwd = to_goal / n
        lat_left = np.array([-fwd[1], fwd[0]])   # +bias = left (consistent with cand_lat)
        for scale in (1.0, 0.5):
            cand = self._target_state[0] + lat_left * (bias * scale)
            if not self._esdf.has_collision(cand):
                self._target_state[0] = cand
                goal_dir = self._global_target - cand
                gn = float(np.linalg.norm(goal_dir))
                self._target_state[1] = (goal_dir / gn * self._v_max * 0.8
                                         if gn > 1e-3 else np.zeros(2))
                break

    def _warm_up(self):
        dummy = np.zeros(IMG_WIDTH * IMG_HEIGHT + MOTION_INPUT_SIZE,
                         dtype=np.float32)
        for _ in range(3):
            self._sample_params(dummy)
        time.sleep(50 / self._cmd_hz + 0.5)

    # ── Anchor state (v2.1) ──────────────────────────────────────────────────
    # No stored waypoints anymore: the anchor is rebuilt every replan from the
    # ACTIVE trajectory remainder (_anchor_from_traj), so _install_trajectory
    # needs no override. Only the escape suppression remains.

    def _enter_escape(self):
        super()._enter_escape()
        # Escape means continuity already broke — suppress the anchor for the
        # next replan and let it resample fully fresh.
        self._anchor_hold = True


def main(args=None):
    rclpy.init(args=args)
    node = FMInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    seq = threading.Thread(target=node.run_sequence, daemon=True)
    seq.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        try:
            node._arm(False)
        except Exception:
            pass
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
