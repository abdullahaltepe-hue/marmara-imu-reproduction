"""
Main reproduction experiment runner.

Pipeline:
  1. Generate the four static-tilt segments (roll +/-15 deg, pitch +/-15 deg)
     from the synthetic IMU data generator.
  2. Run each of the four orientation estimators (Trig, Kalman, Madgwick,
     Mahony) on every segment.
  3. Aggregate accuracy statistics (|mean error|, std) across segments.
  4. Pull per-step compute time estimates from the analytical timing model.
  5. Emit a JSON summary and a markdown table for inclusion in the paper.
  6. Produce three matplotlib figures (saved as PNG):
        fig_timeseries.png  -- estimates vs ground truth at +15 deg roll
        fig_error_hist.png  -- per-algorithm error histograms (steady state)
        fig_compute_time.png -- bar chart of compute-time estimates
"""

from __future__ import annotations
import json
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from synthetic_imu import make_paper_scenario
import algorithms as alg
import timing_model as tm


def run_one_segment(seg, fs):
    """Run all four estimators on one segment."""
    estimators = {
        "Trig":     alg.TrigEstimator(),
        "Kalman":   alg.KalmanEstimator(),
        "Madgwick": alg.MadgwickEstimator(beta=0.1),
        "Mahony":   alg.MahonyEstimator(kp=10.0, ki=0.0),
    }
    out = {}
    for name, est in estimators.items():
        out[name] = alg.run_estimator(est, seg["acc"], seg["gyro"], fs)
    return out


def steady_state_error(est_rad, gt_rad, fs, settle_s=5.0):
    """Compute |mean| and std of (estimate - ground truth) on the
    steady-state portion of the signal (after a settling window)."""
    n0 = int(settle_s * fs)
    err = np.rad2deg(est_rad[n0:] - gt_rad[n0:])
    return float(np.abs(err.mean())), float(err.std())


def aggregate(results, segments, fs):
    """results: dict[seg_label] = dict[algo] = ndarray (N, 2)"""
    rows = {}
    for algo in ["Trig", "Kalman", "Madgwick", "Mahony"]:
        # Pool errors across the four segments
        roll_mu, roll_sd, pitch_mu, pitch_sd = [], [], [], []
        for seg in segments:
            est = results[seg["label"]][algo]
            mu_r, sd_r = steady_state_error(est[:, 0], seg["gt"][:, 0], fs)
            mu_p, sd_p = steady_state_error(est[:, 1], seg["gt"][:, 1], fs)
            roll_mu.append(mu_r); roll_sd.append(sd_r)
            pitch_mu.append(mu_p); pitch_sd.append(sd_p)
        rows[algo] = {
            "roll_abs_mean": float(np.mean(roll_mu)),
            "roll_std":      float(np.mean(roll_sd)),
            "pitch_abs_mean": float(np.mean(pitch_mu)),
            "pitch_std":     float(np.mean(pitch_sd)),
        }
    return rows


def make_timeseries_plot(seg, ests, fs, out_path):
    """Plot ground truth vs all four estimates (roll only) for the given
    +15 deg segment."""
    t = seg["t"]
    fig, ax = plt.subplots(figsize=(7.0, 3.6), dpi=150)
    ax.plot(t, np.rad2deg(seg["gt"][:, 0]), "k--",
            linewidth=1.2, label="Ground truth (+15 deg)")
    colors = {"Trig": "#999999", "Kalman": "#1f77b4",
              "Madgwick": "#ff7f0e", "Mahony": "#2ca02c"}
    for name in ["Trig", "Kalman", "Madgwick", "Mahony"]:
        ax.plot(t, np.rad2deg(ests[name][:, 0]),
                color=colors[name], linewidth=0.9, alpha=0.85, label=name)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Roll estimate [deg]")
    ax.set_title("Roll estimate at static +15 deg tilt (synthetic MPU-6050 noise)")
    ax.set_ylim(13.5, 16.5)
    ax.set_xlim(0, t[-1])
    ax.legend(loc="upper right", fontsize=8, ncol=5)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def make_error_hist(seg, ests, fs, out_path, settle_s=5.0):
    n0 = int(settle_s * fs)
    fig, axs = plt.subplots(2, 2, figsize=(7.0, 5.0), dpi=150,
                            sharex=True, sharey=False)
    colors = {"Trig": "#999999", "Kalman": "#1f77b4",
              "Madgwick": "#ff7f0e", "Mahony": "#2ca02c"}
    for ax, name in zip(axs.flat, ["Trig", "Kalman", "Madgwick", "Mahony"]):
        err = np.rad2deg(ests[name][n0:, 0] - seg["gt"][n0:, 0])
        ax.hist(err, bins=60, color=colors[name],
                edgecolor="black", linewidth=0.3)
        ax.set_title(f"{name}: roll error distribution",
                     fontsize=10)
        ax.set_xlabel("error [deg]")
        ax.axvline(0, color="k", linestyle=":", linewidth=0.8)
        ax.grid(True, alpha=0.3)
    fig.suptitle("Steady-state roll error at +15 deg tilt", fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def make_compute_time_plot(timings, out_path):
    fig, ax = plt.subplots(figsize=(5.5, 3.2), dpi=150)
    names = list(timings.keys())
    values = [timings[n] for n in names]
    colors = ["#999999", "#1f77b4", "#ff7f0e", "#2ca02c"]
    bars = ax.bar(names, values, color=colors, edgecolor="black", linewidth=0.6)
    for b, v in zip(bars, values):
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() * 1.02,
                f"{v:.0f}", ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("Per-step compute time [us]")
    ax.set_title("Estimated per-step cost on ATmega328P @16 MHz")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def main():
    fs = 100.0
    segments, meta = make_paper_scenario(fs=fs, seg_seconds=30.0, seed=42)

    # 1) Run all estimators on every segment
    results = {}
    for seg in segments:
        results[seg["label"]] = run_one_segment(seg, fs)

    # 2) Accuracy aggregation
    accuracy = aggregate(results, segments, fs)

    # 3) Pull timing estimates
    timings = tm.get_estimates()

    # 4) Print summary
    print("\n=== ACCURACY (averaged over 4 static-tilt segments) ===")
    print(f"{'Algorithm':<10} {'|mu|_roll':>11} {'sd_roll':>10} "
          f"{'|mu|_pitch':>12} {'sd_pitch':>10}")
    for algo, r in accuracy.items():
        print(f"{algo:<10} {r['roll_abs_mean']:>10.3f}  "
              f"{r['roll_std']:>9.3f}  {r['pitch_abs_mean']:>11.3f}  "
              f"{r['pitch_std']:>9.3f}")

    print("\n=== PER-STEP COMPUTE TIME (analytical, ATmega328P @16 MHz) ===")
    for algo, t in timings.items():
        print(f"{algo:<10} {t:>8.1f} us  ({1e6/t:>6.0f} Hz max)")

    # 5) Plots
    seg_p15 = next(s for s in segments if s["label"] == "roll+15")
    make_timeseries_plot(seg_p15, results["roll+15"], fs,
                         "fig_timeseries.png")
    make_error_hist(seg_p15, results["roll+15"], fs,
                    "fig_error_hist.png")
    make_compute_time_plot(timings, "fig_compute_time.png")
    print("\nSaved figures: fig_timeseries.png, fig_error_hist.png, "
          "fig_compute_time.png")

    # 6) JSON dump
    summary = {
        "scenario": {
            "fs": fs,
            "segment_seconds": meta["seg_seconds"],
            "acc_sigma_m_s2": meta["acc_sigma"],
            "gyro_sigma_dps": meta["gyro_sigma_dps"],
            "acc_bias_m_s2": meta["acc_bias"].tolist(),
            "gyro_bias_dps": np.rad2deg(meta["gyro_bias"]).tolist(),
        },
        "accuracy_deg": accuracy,
        "compute_time_us": timings,
    }
    with open("results.json", "w") as f:
        json.dump(summary, f, indent=2)
    print("Saved: results.json")


if __name__ == "__main__":
    main()
