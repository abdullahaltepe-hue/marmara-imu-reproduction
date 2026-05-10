# Project Notes — Design Decisions and Reasoning

This document captures *why* the project is shaped the way it is.
It is meant to be read alongside the LaTeX paper, not by itself.
If the paper is the *what*, this is the *why*.

---

## 1. Why a simulation-based reproduction?

We did not have an MPU-6050 module or an Arduino Nano on hand at the
time of writing. The two realistic options were:

- **(A)** Order the hardware (~150-300 TL), wait, build a mechanical
  fixture for known ±15° tilts, run the experiment.
- **(B)** Build a synthetic data pipeline that mimics the MPU-6050,
  re-implement the four algorithms, and time them on the ATmega328P
  with an analytical instruction-count model.

We chose (B) because:

1. We had already written the LaTeX paper as a pre-experimental
   protocol, so the structure was ready to absorb whatever results
   we produced.
2. Simulation gives **exact ground truth** — there is no mechanical
   fixture error to subtract. In the original paper the ±15° tilts
   are mechanically held, which introduces its own ~0.1°-0.5°
   uncertainty.
3. The experiment becomes **deterministic and fully reproducible**
   by anyone with Python; no hardware lottery.
4. The cost of B is that the synthetic noise is Gaussian and
   stationary, which the real silicon is not. This is documented
   in Section VIII of the paper as a Threat to Validity.

Option (A) remains the natural follow-up and would tighten the C5
verdict.

---

## 2. Why these specific noise parameters?

The synthetic accelerometer and gyroscope noise standard deviations
were derived from the MPU-6050 datasheet at default ranges with the
on-chip DLPF set to 44 Hz cutoff:

- **Accelerometer noise density**: ~400 µg/√Hz
  → σ ≈ 400e-6 × 9.81 × √44 ≈ **0.026 m/s²** per sample at 100 Hz
- **Gyroscope noise density**: ~0.005 dps/√Hz
  → σ ≈ **0.05 deg/s** per sample

The bias values are conservative typicals from anecdotal MPU-6050
data:

- Accelerometer bias: σ ≈ 0.05 m/s² (drawn once per session,
  same across all four segments — same physical sensor)
- Gyroscope bias: σ ≈ 0.5 deg/s (same)

These are best-effort approximations. If a real-hardware run
produces different bias floors, the LaTeX paper's tables can be
regenerated with the new numbers without changing the structure.

---

## 3. Why these algorithm parameters?

| Parameter      | Value         | Source                          |
|----------------|---------------|---------------------------------|
| Mahony Kp      | 10            | Badia et al. directly           |
| Mahony Ki      | 0             | Badia et al. directly           |
| Madgwick β     | 0.1           | Madgwick (2010) tech report default |
| Kalman Q angle | 1e-3          | sensible default                |
| Kalman Q bias  | 1e-5          | sensible default                |
| Kalman R       | 0.03          | sensible default                |
| Sample rate    | 100 Hz        | matches Badia et al.            |

The Mahony gains are the only algorithm parameters Badia et al.
explicitly publish. For the rest we used primary-reference defaults
and noted in the paper that a tuning sweep is left for future work.

The fact that **Mahony at Kp=10 outperforms Kalman at default
(Q,R)** in our σ measurement is a real consequence of these choices
— Kp=10 is fairly aggressive low-pass filtering, while default Q/R
on the Kalman filter is a fairly trusting balance. A different
tuning would shift the ranking. We chose to leave the values exactly
as in the references rather than re-tune to flatter Kalman.

---

## 4. Why the analytical timing model?

Without a real Arduino in hand we cannot run a `micros()` benchmark.
Three alternatives existed:

- **simavr** (cycle-accurate AVR emulator): not installable in our
  sandbox without sudo.
- **Wokwi** (web Arduino simulator): possible but timing fidelity
  not fully verified.
- **Analytical instruction count**: count the float operations in
  each algorithm by hand, multiply by published ATmega328P
  per-operation cycle costs, divide by 16 MHz.

The analytical model is the most transparent and the most easily
checked: anyone can read `timing_model.py`, agree or disagree with
the per-op cycle costs, and re-derive the table. The absolute
numbers carry a ~±20% uncertainty, but the **rank ordering
between the three filters** is robust to that band — Madgwick
moves through 6× more `fmul` operations than Trig does, and that
gap dominates any reasonable per-op cost variation.

---

## 5. The Trig-vs-filter compute-time divergence

Our model says Trig is the cheapest (568 µs), Badia et al. say
it is the most expensive. Three reasons this might be:

1. **What gets timed.** On a real Nano, "trig per-step time" often
   bundles the I²C accelerometer read and any final
   degree/radian conversion. Our model times only the angle
   computation itself.
2. **`atan2f` implementation.** avr-libc's `atan2f` cycle cost
   varies between toolchain versions; our 4000-cycle figure is a
   typical published value but can be 2× higher in some versions.
3. **Soft-float library calibration.** Per-operation costs are
   averages over operand distributions; for the specific operand
   ranges in static-tilt accelerometer data, real costs may
   shift.

We chose to **report the divergence honestly** rather than tune
the model into agreement, because reproduction studies that
silently calibrate to confirm the source paper are not useful.
The right next step is a real-hardware `micros()` run that pins
down which of the three explanations is correct.

---

## 6. Why this report length and structure?

The paper sits at 7 pages in IEEE 2-column conference format —
the standard "short paper" length for ISITIA, IECON, SIU and
similar venues. The structure follows the canonical
Introduction → Background → Method → Results → Discussion →
Threats → Conclusion flow that journal and conference editors
expect. Nothing here is unusual; that's deliberate, because
reproduction studies should look like ordinary papers, not like
oddities.

---

## 7. What's the natural next version?

If a real MPU-6050 + Arduino Nano become available, the
experiment runner can absorb a real CSV log with no structural
changes. The LaTeX numbers would update in:

- Table III (accuracy): replace synthetic σ values with measured ones
- Table IV (compute time): replace analytical estimates with
  `micros()` averages

The Threats to Validity section would shorten, and the C5
verdict would either flip to "confirmed" or stay "diverged",
with an empirical answer instead of a model-based one.

That's a 1-2 day extension once hardware is in hand.
