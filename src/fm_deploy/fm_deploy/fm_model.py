#!/usr/bin/env python3
# fm_model.py — v1.0 — Flow Matching planner model (encoder identical to the NN baseline's PlannerNet, output (Q, t̄) MINCO)
"""
FM planner — generative multimodal initializer for MINCO optimization.
=============================================================================
Design (per research discussion):
  - Learns the conditional DISTRIBUTION p(x1 | o) of expert solutions instead
    of the conditional mean (MSE regressor -> mode averaging -> collision on
    wide/symmetric obstacles). Rectified-flow style Conditional Flow Matching:
        x_t = (1 - t) x0 + t x1,   target velocity = x1 - x0,
        loss = || v_theta(x_t, t, c) - (x1 - x0) ||^2
    Straight interpolation paths -> accurate sampling with 1-2 Euler steps
    (onboard-feasible on Jetson).

  - Blocks (canonical FM-planner anatomy):
      ObservationEncoder : SAME backbones as PlannerNet (ResNet18 1-ch frozen
                           + fc->24; motion MLP [48,24,24]->24) -> context 48.
                           Run ONCE per replanning, shared by all K samples
                           and all Euler steps. Identical encoder = clean
                           apple-to-apple ablation vs the NN-Planner.
      TimeEmbedding      : sinusoidal(32) + small MLP.
      VelocityField      : MLP on concat[x_t(9), t_emb(32), context(48)].
      ParamNormalizer    : per-dim standardization of the 9-dim output
                           (wpts meters vs ts seconds live on different
                           scales; FM assumes a well-scaled target space).

  - I/O convention == deployment node (fm_inference_node._initial_guesses):
      input  : flat [depth_u8 as float32 (0..255, NO /255), motion_info(24)]
      output : 9 = [w1_body_xyz, w2_body_xyz, ts1..ts3]  (denormalized)
      The node rotates wpts to world frame and clips ts to [T_min, T_max].

DATA REQUIREMENT (blocking for full capability):
    The current expert saves only the LOWEST-COST solution per scene ->
    effectively unimodal dataset. FM trained on it already fixes
    mode-averaging failures on ambiguous scenes (<=1.2 m), but CANNOT invent
    the missing detour modes for obstacles > 1.2 m. Modify
    expert_planner_node.py to save ALL converged feasible solutions per scene
    (multi-homotopy front-end) before expecting wide-obstacle gains.
"""
import json
import math
import os

import numpy as np
import torch
import torch.nn as nn
import torchvision.models as models

IMG_WIDTH = 640
IMG_HEIGHT = 480
MOTION_INPUT_SIZE = 24
PARAM_DIM = 9            # [w1_xyz, w2_xyz, ts1..3] body frame

# 2026-07-14 (v1.1 visual128): IMG_FEATURE_SIZE 24 -> 128 + layer3/4 ResNet
# unfrozen. Offline diagnosis: both models' error was ~10x larger on scenes
# whose answer depends on the depth image content VS free scenes, even on
# the training data itself -> obstacle information was barely getting past
# the 24-dim bottleneck from the frozen backbone. The old checkpoint (24)
# can still be loaded: load_checkpoint() detects the feature size from the
# state_dict.
IMG_FEATURE_SIZE = 128   # 24 (== PlannerNet) -> 128; the NN baseline is left as-is
MOTION_FEATURE_SIZE = 24  # == PlannerNet
# NOTE (2026-07-27): PlannerNet itself no longer lives in this package — the NN
# planner was removed. It survives in the training repo (~/Training_model_FM_planner/
# nn_trainer.py); the references above describe the shared encoder lineage.
CONTEXT_DIM = IMG_FEATURE_SIZE + MOTION_FEATURE_SIZE  # 152
TIME_EMB_DIM = 32
VF_HIDDEN = 128


# ─────────────────────────────────────────────────────────────────────────────
# Normalizer (differentiable — needed later by Stage-2 cost fine-tuning)
# ─────────────────────────────────────────────────────────────────────────────
class ParamNormalizer(nn.Module):
    """z = (x - mean) / std, per output dim, stored as buffers."""

    def __init__(self, dim=PARAM_DIM):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("std", torch.ones(dim))

    def fit(self, samples_np):
        m = samples_np.mean(axis=0)
        s = samples_np.std(axis=0)
        s = np.where(s < 1e-6, 1.0, s)  # guard constant dims (e.g. body z ~ 0)
        self.mean.copy_(torch.from_numpy(m.astype(np.float32)))
        self.std.copy_(torch.from_numpy(s.astype(np.float32)))

    def normalize(self, x):
        return (x - self.mean) / self.std

    def denormalize(self, z):
        return z * self.std + self.mean

    def save_json(self, path):
        with open(path, "w") as f:
            json.dump({"mean": self.mean.tolist(),
                       "std": self.std.tolist()}, f, indent=2)

    def load_json(self, path):
        with open(path) as f:
            d = json.load(f)
        self.mean.copy_(torch.tensor(d["mean"], dtype=torch.float32))
        self.std.copy_(torch.tensor(d["std"], dtype=torch.float32))


# ─────────────────────────────────────────────────────────────────────────────
# Block 1 — Observation encoder (backbone identical to PlannerNet)
# ─────────────────────────────────────────────────────────────────────────────
class ObservationEncoder(nn.Module):
    def __init__(self, pretrained=True, img_feature=IMG_FEATURE_SIZE):
        super().__init__()
        self.img_feature = img_feature
        if pretrained:
            try:
                self.img_backbone = models.resnet18(weights="DEFAULT")
            except Exception as e:
                print(f"[WARN] pretrained ResNet18 unavailable ({e}) "
                      "— random init")
                self.img_backbone = models.resnet18(weights=None)
        else:
            self.img_backbone = models.resnet18(weights=None)

        for param in self.img_backbone.parameters():
            param.requires_grad = False  # freeze layer1/2 (generic features)

        # v1.1: layer3/4 also train — the high-level ImageNet features need
        # to adapt to depth images. Train with a smaller LR than the head
        # (see the param group in fm_trainer.py). Does not change the
        # architecture, so checkpoint compatibility is unaffected.
        for blk in (self.img_backbone.layer3, self.img_backbone.layer4):
            for param in blk.parameters():
                param.requires_grad = True

        self.img_backbone.conv1 = nn.Conv2d(
            1, 64, kernel_size=7, stride=2, padding=3, bias=False)
        self.img_backbone.fc = nn.Linear(
            self.img_backbone.fc.in_features, img_feature)

        self.motion_backbone = nn.Sequential(
            nn.Linear(MOTION_INPUT_SIZE, 48), nn.LeakyReLU(),
            nn.Linear(48, 24), nn.LeakyReLU(),
            nn.Linear(24, 24), nn.LeakyReLU(),
            nn.Linear(24, MOTION_FEATURE_SIZE),
        )

    def forward(self, flat_input):
        """flat_input: [B, 640*480+24] (same convention as PlannerNet)."""
        img = flat_input[:, :IMG_WIDTH * IMG_HEIGHT].reshape(
            -1, 1, IMG_HEIGHT, IMG_WIDTH)
        vector = flat_input[:, IMG_WIDTH * IMG_HEIGHT:]
        img_feature = self.img_backbone(img)
        motion_feature = self.motion_backbone(vector)
        return torch.cat([img_feature, motion_feature], dim=1)  # [B, 48]


# ─────────────────────────────────────────────────────────────────────────────
# Block 2 — Sinusoidal time embedding
# ─────────────────────────────────────────────────────────────────────────────
class TimeEmbedding(nn.Module):
    def __init__(self, dim=TIME_EMB_DIM):
        super().__init__()
        self.dim = dim
        self.mlp = nn.Sequential(
            nn.Linear(dim, dim), nn.LeakyReLU(), nn.Linear(dim, dim))

    def forward(self, t):
        """t: [B] in [0,1] -> [B, dim]"""
        half = self.dim // 2
        freqs = torch.exp(
            -math.log(10000.0)
            * torch.arange(half, device=t.device, dtype=torch.float32) / half)
        angles = t[:, None] * freqs[None, :] * 1000.0
        emb = torch.cat([torch.sin(angles), torch.cos(angles)], dim=1)
        return self.mlp(emb)


# ─────────────────────────────────────────────────────────────────────────────
# Block 3 — Velocity field v_theta(x_t, t, c)
# ─────────────────────────────────────────────────────────────────────────────
class VelocityField(nn.Module):
    def __init__(self, context_dim=CONTEXT_DIM):
        super().__init__()
        in_dim = PARAM_DIM + TIME_EMB_DIM + context_dim
        self.net = nn.Sequential(
            nn.Linear(in_dim, VF_HIDDEN), nn.LeakyReLU(),
            nn.Linear(VF_HIDDEN, VF_HIDDEN), nn.LeakyReLU(),
            nn.Linear(VF_HIDDEN, VF_HIDDEN), nn.LeakyReLU(),
            nn.Linear(VF_HIDDEN, PARAM_DIM),
        )

    def forward(self, x_t, t_emb, context):
        return self.net(torch.cat([x_t, t_emb, context], dim=1))


# ─────────────────────────────────────────────────────────────────────────────
# Block 4 — FM planner: CFM loss (training) + Euler sampling (inference)
# ─────────────────────────────────────────────────────────────────────────────
class FMPlanner(nn.Module):
    def __init__(self, pretrained_encoder=True, img_feature=IMG_FEATURE_SIZE):
        super().__init__()
        self.encoder = ObservationEncoder(pretrained=pretrained_encoder,
                                          img_feature=img_feature)
        self.time_emb = TimeEmbedding()
        self.velocity = VelocityField(
            context_dim=img_feature + MOTION_FEATURE_SIZE)
        self.normalizer = ParamNormalizer()

    # ── Training (Stage 1: imitation) ───────────────────────────────────────
    def cfm_loss(self, flat_input, target_params):
        """Rectified-flow CFM loss.
        flat_input    : [B, 640*480+24]
        target_params : [B, 9] RAW (meters/seconds); normalized internally.
        """
        context = self.encoder(flat_input)                     # [B, 48]
        x1 = self.normalizer.normalize(target_params)          # [B, 9]
        x0 = torch.randn_like(x1)
        t = torch.rand(x1.shape[0], device=x1.device)
        x_t = (1.0 - t[:, None]) * x0 + t[:, None] * x1
        v_target = x1 - x0
        v_pred = self.velocity(x_t, self.time_emb(t), context)
        return torch.mean((v_pred - v_target) ** 2)

    # ── Inference ────────────────────────────────────────────────────────────
    @torch.no_grad()
    def sample(self, flat_input, K=8, n_steps=2, generator=None):
        """K candidates for ONE observation.
        flat_input: [1, 640*480+24]  ->  returns [K, 9] RAW params.
        Encoder runs ONCE; velocity field runs n_steps times, K batched.
        """
        context = self.encoder(flat_input)                  # [1, 48]
        context = context.expand(K, -1)                     # shared
        x = torch.randn(K, PARAM_DIM, device=flat_input.device,
                        generator=generator)
        dt = 1.0 / n_steps
        for i in range(n_steps):
            t = torch.full((K,), i * dt, device=x.device)
            x = x + dt * self.velocity(x, self.time_emb(t), context)
        return self.normalizer.denormalize(x)

    def sample_differentiable(self, context, K, n_steps=2):
        """Same Euler chain WITH gradients — used by Stage-2 cost fine-tuning
        (backprop path: H -> denormalize -> Euler -> velocity -> encoder)."""
        x = torch.randn(K, PARAM_DIM, device=context.device)
        ctx = context.expand(K, -1) if context.shape[0] == 1 else context
        dt = 1.0 / n_steps
        for i in range(n_steps):
            t = torch.full((K,), i * dt, device=x.device)
            x = x + dt * self.velocity(x, self.time_emb(t), ctx)
        return self.normalizer.denormalize(x)

    # ── Deployment helper (mirrors the fm_inference_node convention) ────────
    @torch.no_grad()
    def predict_candidates(self, flat_input_np, K=8, n_steps=2,
                           device="cpu"):
        """numpy in -> list of (int_wpts_body[3,2-like 9dim], ts) candidates.
        Returns raw [K, 9]; the ROS node applies the SAME post-processing it
        already applies to PlannerNet output (rotate to world, clip ts)."""
        x = torch.from_numpy(
            flat_input_np.astype(np.float32)).unsqueeze(0).to(device)
        out = self.sample(x, K=K, n_steps=n_steps)
        return out.cpu().numpy()


# ─────────────────────────────────────────────────────────────────────────────
# ONNX export — single graph: flat input + noise -> K candidates
# ─────────────────────────────────────────────────────────────────────────────
class _FMExportWrapper(nn.Module):
    """Unrolled n_steps Euler for ONNX (no python loop state at runtime)."""

    def __init__(self, fm: FMPlanner, n_steps=2):
        super().__init__()
        self.fm = fm
        self.n_steps = n_steps

    def forward(self, flat_input, noise):
        """flat_input: [1, 640*480+24], noise: [K, 9] -> [K, 9] raw params."""
        context = self.fm.encoder(flat_input)
        K = noise.shape[0]
        ctx = context.expand(K, -1)
        x = noise
        dt = 1.0 / self.n_steps
        for i in range(self.n_steps):
            t = torch.full((K,), i * dt, device=x.device)
            x = x + dt * self.fm.velocity(x, self.fm.time_emb(t), ctx)
        return self.fm.normalizer.denormalize(x)


def export_onnx(fm: FMPlanner, path, K=8, n_steps=2, device="cpu"):
    fm = fm.to(device).eval()
    wrapper = _FMExportWrapper(fm, n_steps=n_steps).to(device).eval()
    dummy_in = torch.randn(1, IMG_WIDTH * IMG_HEIGHT + MOTION_INPUT_SIZE,
                           device=device)
    dummy_noise = torch.randn(K, PARAM_DIM, device=device)
    try:
        torch.onnx.export(
            wrapper, (dummy_in, dummy_noise), path,
            input_names=["input", "noise"], output_names=["candidates"],
            dynamic_axes={"noise": {0: "K"}, "candidates": {0: "K"}})
        print("onnx model saved:", path)
    except Exception as e:  # onnx deps missing -> keep the .pth result
        print(f"[WARN] ONNX export failed ({e}); .pth is unaffected")


# ─────────────────────────────────────────────────────────────────────────────
# Checkpoint helpers (model + normalizer bundled together)
# ─────────────────────────────────────────────────────────────────────────────
def save_checkpoint(fm: FMPlanner, path):
    torch.save({"state_dict": fm.state_dict(),
                "norm_mean": fm.normalizer.mean.cpu().numpy().tolist(),
                "norm_std": fm.normalizer.std.cpu().numpy().tolist()}, path)
    print("pth model saved:", path)


def load_checkpoint(path, device="cpu", pretrained_encoder=False):
    ckpt = torch.load(path, map_location=device)
    # v1.1: the image feature size is read from the checkpoint itself, so
    # both the old (24) and new (128) checkpoints load without a flag.
    img_feature = int(
        ckpt["state_dict"]["encoder.img_backbone.fc.weight"].shape[0])
    fm = FMPlanner(pretrained_encoder=pretrained_encoder,
                   img_feature=img_feature)
    fm.load_state_dict(ckpt["state_dict"])
    return fm.to(device)
