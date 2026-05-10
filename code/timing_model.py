"""
ATmega328P float-instruction-count timing model for the four
orientation estimators.

The Arduino Nano runs an 8-bit ATmega328P at 16 MHz. Single-precision
floating-point operations are emulated by the avr-libc library; the
typical published cycle counts (averages over operand distributions)
are used here. Cycle counts vary somewhat with operand magnitude and
compiler version, so the absolute numbers should be read as
ESTIMATES (within roughly +/- 20%) and the principal deliverable is
the RANK ORDERING of the four algorithms.

Sources for the per-operation cycle costs:
- avr-libc benchmarks (libm, libgcc soft-float),
- "AVR4027" Atmel application note,
- Arduino forums thread on float performance.

Conservative published values used here (single-precision):
    fadd / fsub : 90 cycles
    fmul        : 100 cycles
    fdiv        : 500 cycles
    sqrtf       : 700 cycles
    atan2f      : 4000 cycles
    asinf       : 4000 cycles
    sinf / cosf : 2500 cycles

Clock period at 16 MHz: 62.5 ns / cycle.
"""

from __future__ import annotations
from dataclasses import dataclass

CLK_HZ = 16_000_000
NS_PER_CYCLE = 1e9 / CLK_HZ

CYCLES = {
    "fadd": 90,
    "fmul": 100,
    "fdiv": 500,
    "sqrt": 700,
    "atan2": 4000,
    "asin": 4000,
    "sincos": 2500,
}


@dataclass
class OpCount:
    fadd: int = 0
    fmul: int = 0
    fdiv: int = 0
    sqrt: int = 0
    atan2: int = 0
    asin: int = 0
    sincos: int = 0

    def cycles(self) -> int:
        return (
            self.fadd * CYCLES["fadd"]
            + self.fmul * CYCLES["fmul"]
            + self.fdiv * CYCLES["fdiv"]
            + self.sqrt * CYCLES["sqrt"]
            + self.atan2 * CYCLES["atan2"]
            + self.asin * CYCLES["asin"]
            + self.sincos * CYCLES["sincos"]
        )

    def microseconds(self) -> float:
        return self.cycles() * NS_PER_CYCLE / 1000.0


# ---------------------------------------------------------------------------
# Per-step instruction counts for each algorithm.
# Counts include only the FILTER STEP itself (not sensor I/O, not the
# optional final quaternion-to-euler conversion).
# ---------------------------------------------------------------------------

# Trig: 2x atan2(...) + sqrt(ay^2+az^2)
TRIG = OpCount(
    fmul=3,    # ay*ay, az*az, -ax (just sign flip; counted)
    fadd=1,    # ay*ay + az*az
    sqrt=1,    # sqrt(...)
    atan2=2,   # roll, pitch
)

# Kalman 1D Euler, single-axis update (with bias state):
#   predict: F*x (2 mul + 1 add) + Q add
#   F*P*F^T  : ~8 mul + 4 add
#   K = P[:,0] / (P[0,0]+R) : 1 add + 2 div
#   innovation y, x += K*y  : 2 add + 2 mul
#   P update : ~6 mul + 4 add
KALMAN_AXIS = OpCount(fmul=18, fadd=12, fdiv=2)

# Trig measurement once for roll, once for pitch (atan2 + (sqrt path for pitch))
KALMAN_MEAS = OpCount(fmul=3, fadd=1, sqrt=1, atan2=2)

# Two parallel Kalman axes + one shared trig measurement step
KALMAN = OpCount(
    fmul=KALMAN_MEAS.fmul + 2 * KALMAN_AXIS.fmul,
    fadd=KALMAN_MEAS.fadd + 2 * KALMAN_AXIS.fadd,
    fdiv=KALMAN_MEAS.fdiv + 2 * KALMAN_AXIS.fdiv,
    sqrt=KALMAN_MEAS.sqrt,
    atan2=KALMAN_MEAS.atan2,
)

# Madgwick (6-axis) per-step:
#   gyro qDot       : 8 mul + 8 add (with the *0.5 factored)
#   acc norm        : 3 mul + 2 add + 1 sqrt
#   acc normalize   : 3 div
#   auxiliary vars  : ~12 mul
#   gradient s1..s4 : ~25 mul + 15 add
#   gradient norm   : 4 mul + 3 add + 1 sqrt + 4 div
#   subtract beta*s : 4 mul + 4 add
#   integrate qDot  : 4 mul + 4 add
#   normalize quat  : 4 mul + 3 add + 1 sqrt + 4 div
MADGWICK = OpCount(
    fmul=8 + 3 + 12 + 25 + 4 + 4 + 4 + 4,
    fadd=8 + 2 + 15 + 3 + 4 + 4 + 3,
    fdiv=3 + 4 + 4,
    sqrt=3,
)

# Mahony (6-axis, ki=0) per-step:
#   acc norm        : 3 mul + 2 add + 1 sqrt
#   acc normalize   : 3 div
#   predicted gravity: 6 mul + 3 add
#   error cross prod : 6 mul + 3 add
#   proportional add : 3 mul + 3 add
#   gyro qDot        : 8 mul + 8 add
#   integrate qDot   : 4 mul + 4 add
#   normalize quat   : 4 mul + 3 add + 1 sqrt + 4 div
MAHONY = OpCount(
    fmul=3 + 6 + 6 + 3 + 8 + 4 + 4,
    fadd=2 + 3 + 3 + 3 + 8 + 4 + 3,
    fdiv=3 + 4,
    sqrt=2,
)


def report():
    print(f"{'Algorithm':<12} {'fmul':>5} {'fadd':>5} {'fdiv':>5} "
          f"{'sqrt':>5} {'atan2':>6} {'cycles':>9} {'us':>9}")
    for name, oc in [("Trig", TRIG), ("Kalman", KALMAN),
                     ("Madgwick", MADGWICK), ("Mahony", MAHONY)]:
        print(f"{name:<12} {oc.fmul:>5} {oc.fadd:>5} {oc.fdiv:>5} "
              f"{oc.sqrt:>5} {oc.atan2:>6} {oc.cycles():>9} "
              f"{oc.microseconds():>9.1f}")


def get_estimates() -> dict:
    return {
        "Trig":     TRIG.microseconds(),
        "Kalman":   KALMAN.microseconds(),
        "Madgwick": MADGWICK.microseconds(),
        "Mahony":   MAHONY.microseconds(),
    }


if __name__ == "__main__":
    report()
