# Marmara IMU Reproduction Project

A simulation-based reproduction of:

> S. Sansone, N. Bartolini, G. Perin, L. Badia,
> *"A Comparative Analysis of Sensor Fusion Algorithms for Miniature
> IMU Measurements,"* ISITIA 2023, IEEE.

**Team:** Abdullah Altepe, Mehmet Bahçeci, Yusuf Oruç, Murat Demir
**Institution:** Marmara University, Department of Electrical and Electronics Engineering

---

## What's in this folder

```
marmara_imu_reproduction/
├── README.md                  <-- you are here
├── notes.md                   <-- design decisions and reasoning
├── code/
│   ├── synthetic_imu.py       MPU-6050 noise/bias model + ±15° tilt scenarios
│   ├── algorithms.py          Trig, Kalman 1D, Madgwick, Mahony in pure Python
│   ├── timing_model.py        ATmega328P float-instruction-count timing model
│   ├── run_experiment.py      Main runner: produces tables + figures
│   └── results.json           All numerical outputs (accuracy + compute time)
├── figures/
│   ├── fig_timeseries.png     4 algorithms vs ground truth at +15° roll
│   ├── fig_error_hist.png     Steady-state error histograms
│   └── fig_compute_time.png   Per-step compute-time bar chart
├── paper/
│   ├── main.tex               IEEE conference paper LaTeX source (7 pages)
│   └── main_preview.pdf       Sandbox-rendered PDF (real IEEE 2-column on Overleaf)
└── presentation/
    ├── presentation.pptx      18-slide project presentation (English, ~12-15 min)
    └── presentation.pdf       Same deck pre-rendered to PDF for quick preview
```

The presentation is written for an audience that has NOT studied sensor
fusion before. It walks through:

1.  What "orientation" means and why it matters
2.  The MPU-6050 hardware and the two sensors inside
3.  Why neither sensor works alone — the fusion idea
4.  The four algorithms compared by Badia et al.
5.  The reference paper and its claims
6.  Our challenge (no hardware) and the simulation approach
7.  How we built it (four small Python modules)
8.  Results — accuracy and per-step compute time
9.  The one surprising divergence
10. Verdict and next steps

---

## How to reproduce

### 1. Run the simulation experiment (Python)

Requires Python 3.10+ with `numpy`, `scipy`, `matplotlib`.

```bash
cd code
pip install numpy scipy matplotlib
python3 run_experiment.py
```

This regenerates:
- `results.json`
- `fig_timeseries.png`, `fig_error_hist.png`, `fig_compute_time.png`

The run is deterministic (seed = 42), so the numbers in `results.json`
will reproduce exactly.

### 2. Compile the paper (LaTeX)

The recommended workflow is **Overleaf**:

1. Create a new project, upload `paper/main.tex` and the three PNGs from `figures/`.
2. Make sure the document class is `IEEEtran` (built into Overleaf).
3. Click "Recompile" — produces a 7-page IEEE 2-column conference paper.

Local compile also works if you have `texlive-publishers` and `texlive-pictures` installed:

```bash
cd paper
cp ../figures/*.png .
pdflatex main.tex && pdflatex main.tex
```

---

## Headline result

Of the **five qualitative claims** of Badia et al. that we tested:

| # | Claim                                                  | Verdict |
|---|--------------------------------------------------------|---------|
| C1 | Trig has the largest noise σ                          | ✅ Confirmed (σ_Trig = 0.154°) |
| C2 | Kalman ≈ Madgwick in compute (within 20%)             | ✅ Confirmed (4% gap) |
| C3 | Mahony fastest among the three filters                 | ✅ Confirmed (1.6× faster) |
| C4 | Kalman among the lowest-σ algorithms                   | ✅ Confirmed (σ_Kalman = 0.046°) |
| C5 | Trig is the most expensive method overall (implicit)  | ❌ Diverges (in our model Trig is cheapest) |

The C5 divergence is discussed honestly in the paper, Section VII.

---

## Citing

If you use this code or the paper, please cite both the original work
and this reproduction:

```
@inproceedings{badia2023,
  title={A Comparative Analysis of Sensor Fusion Algorithms
         for Miniature IMU Measurements},
  author={Sansone, S. and Bartolini, N. and Perin, G. and Badia, L.},
  booktitle={ISITIA},
  year={2023},
  publisher={IEEE}
}

@misc{marmara2026reproduction,
  title={A Simulation-Based Reproduction of Comparative Sensor
         Fusion Algorithms on a Miniature IMU},
  author={Altepe, A. and Bah{\c{c}}eci, M. and Oru{\c{c}}, Y. and Demir, M.},
  institution={Marmara University, Department of EEE},
  year={2026}
}
```
