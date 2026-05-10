"""
Synthetic IMU Data Generator for the Badia et al. (ISITIA 2023)
reproduction study.

Generates noise-free ground-truth orientation trajectories together
with realistic accelerometer and gyroscope measurements that mimic
the InvenSense MPU-6050 at default ranges (acc +/- 2 g, gyro +/- 250 deg/s).

The static-tilt scenarios used by the reference paper are reproduced
here: the IMU is held first at +15 deg, then at -15 deg, on each axis
(roll and pitch), for 30 s at 100 Hz.

All angles are stored in radians internally; degree conversions happen
only at the I/O boundary.

Author: A.A., M.B., Y.O., M.D. (Marmara University EEE)
"""

from __future__ import annotations
import numpy as np

GRAVITY = 9.80665  # m/s^2

# ---------------------------------------------------------------------------
# MPU-6050 noise model parameters (from datasheet, conservative values)
# ---------------------------------------------------------------------------
# Accelerometer: noise spectral density ~400 ug/sqrt(Hz) at +/- 2g range,
# bandwidth ~44 Hz (DLPF on) -> noise sigma ~= 400e-6 * 9.80665 * sqrt(44)
ACC_SIGMA = 0.026  # m/s^2 per sample at ~100 Hz with DLPF=44Hz
ACC_BIAS_STD = 0.05  # constant per-session bias, m/s^2

# Gyroscope: noise density ~0.005 dps/sqrt(Hz), with DLPF -> sigma ~= 0.05 dps
GYRO_SIGMA_DPS = 0.05  # deg/s per sample
GYRO_SIGMA = np.deg2rad(GYRO_SIGMA_DPS)
GYRO_BIAS_DPS = 0.5  # constant gyro bias, deg/s (typical, varies +/- a few)
GYRO_BIAS = np.deg2rad(GYRO_BIAS_DPS)


def euler_to_gravity_body(roll: float, pitch: float) -> np.ndarray:
    """Project the world gravity vector [0,0,g] into the body frame given
    intrinsic Z-Y-X (yaw-pitch-roll) Euler angles with yaw=0."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    # Body-frame gravity (assuming yaw = 0): R^T * [0,0,g]
    ax = -GRAVITY * sp
    ay = GRAVITY * sr * cp
    az = GRAVITY * cr * cp
    return np.array([ax, ay, az])


def make_static_tilt_segment(
    roll_deg: float,
    pitch_deg: float,
    duration_s: float,
    fs: float = 100.0,
    rng: np.random.Generator | None = None,
    acc_bias: np.ndarray | None = None,
    gyro_bias: np.ndarray | None = None,
):
    """Generate a single static-tilt segment.

    Returns:
        t      (N,)   time vector in seconds
        gt     (N,2)  ground-truth (roll, pitch) in radians
        acc    (N,3)  noisy accelerometer in m/s^2
        gyro   (N,3)  noisy gyroscope in rad/s
    """
    if rng is None:
        rng = np.random.default_rng()

    n = int(duration_s * fs)
    t = np.arange(n) / fs

    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)

    g_body = euler_to_gravity_body(roll, pitch)
    acc_clean = np.tile(g_body, (n, 1))
    gyro_clean = np.zeros((n, 3))  # static -> zero angular rate

    if acc_bias is None:
        acc_bias = rng.normal(0.0, ACC_BIAS_STD, size=3)
    if gyro_bias is None:
        # one constant bias per axis, drawn from N(0, GYRO_BIAS)
        gyro_bias = rng.normal(0.0, GYRO_BIAS, size=3)

    acc_noise = rng.normal(0.0, ACC_SIGMA, size=(n, 3))
    gyro_noise = rng.normal(0.0, GYRO_SIGMA, size=(n, 3))

    acc = acc_clean + acc_bias[None, :] + acc_noise
    gyro = gyro_clean + gyro_bias[None, :] + gyro_noise

    gt = np.tile([roll, pitch], (n, 1))
    return t, gt, acc, gyro, acc_bias, gyro_bias


def make_paper_scenario(
    fs: float = 100.0, seg_seconds: float = 30.0, seed: int = 0
):
    """Reproduce the four reference tilts of the Badia paper:
    (+15 deg roll, 0), (-15 deg roll, 0), (0, +15 deg pitch), (0, -15 deg pitch).

    A short transition window is NOT inserted because the reference paper's
    accuracy comparison is reported on the steady-state portion of each
    static recording.
    """
    rng = np.random.default_rng(seed)
    # Use the SAME bias vector across all four segments (same physical sensor).
    acc_bias = rng.normal(0.0, ACC_BIAS_STD, size=3)
    gyro_bias = rng.normal(0.0, GYRO_BIAS, size=3)

    segments = []
    for label, r, p in [
        ("roll+15", +15.0, 0.0),
        ("roll-15", -15.0, 0.0),
        ("pitch+15", 0.0, +15.0),
        ("pitch-15", 0.0, -15.0),
    ]:
        t, gt, acc, gyro, _, _ = make_static_tilt_segment(
            r, p, seg_seconds, fs, rng, acc_bias=acc_bias, gyro_bias=gyro_bias
        )
        segments.append({
            "label": label,
            "roll_deg": r,
            "pitch_deg": p,
            "t": t,
            "gt": gt,
            "acc": acc,
            "gyro": gyro,
        })
    meta = {
        "fs": fs,
        "seg_seconds": seg_seconds,
        "acc_bias": acc_bias,
        "gyro_bias": gyro_bias,
        "acc_sigma": ACC_SIGMA,
        "gyro_sigma_dps": GYRO_SIGMA_DPS,
    }
    return segments, meta


if __name__ == "__main__":
    segs, meta = make_paper_scenario(seed=42)
    print("Synthetic IMU scenario (Badia reproduction):")
    print(f"  fs                 = {meta['fs']} Hz")
    print(f"  segment duration   = {meta['seg_seconds']} s")
    print(f"  acc bias (m/s^2)   = {meta['acc_bias']}")
    print(f"  gyro bias (deg/s)  = {np.rad2deg(meta['gyro_bias'])}")
    print(f"  acc sigma          = {meta['acc_sigma']} m/s^2")
    print(f"  gyro sigma         = {meta['gyro_sigma_dps']} deg/s")
    for s in segs:
        n = len(s["t"])
        print(f"  segment {s['label']:8s}: {n} samples, "
              f"roll={s['roll_deg']:+5.1f} deg, pitch={s['pitch_deg']:+5.1f} deg")
