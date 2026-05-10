"""
Reference reproductions of the four orientation-estimation algorithms
compared in Sansone, Bartolini, Perin, Badia (ISITIA 2023).

All implementations operate sample-by-sample (no NumPy vectorisation
across time) so that they mirror what the Arduino C++ implementation
would do, and so that operation counts for the analytical timing model
can be derived directly from the source.

Conventions:
- accelerometer  acc  in m/s^2, body frame, [ax, ay, az]
- gyroscope      gyro in rad/s,  body frame, [gx, gy, gz]
- output (roll, pitch) returned in radians
- quaternion convention: q = [w, x, y, z], scalar-first, unit norm
"""

from __future__ import annotations
import math
import numpy as np


# ---------------------------------------------------------------------------
#  Helper: quaternion <-> Euler conversion (Z-Y-X intrinsic, yaw=psi)
# ---------------------------------------------------------------------------
def quat_to_euler(q):
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


# ===========================================================================
#  Algorithm 1: Trigonometric (no fusion, accelerometer only)
# ===========================================================================
class TrigEstimator:
    """Roll/pitch directly from gravity projection.
       Drift-free in the static case, but reproduces every bit of
       accelerometer noise into the angle estimate."""
    def __init__(self):
        pass

    def update(self, acc, gyro=None, dt=None):
        ax, ay, az = acc
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        return roll, pitch


# ===========================================================================
#  Algorithm 2: Kalman filter (1D Euler form per axis, with bias state)
# ===========================================================================
class KalmanAxis:
    """Two-state Kalman filter (angle + gyro bias) for one axis.

    State x = [theta, b]^T,
        F = [[1, -dt],[0, 1]], B = [dt, 0]^T, H = [1, 0]
    """
    def __init__(self, q_angle=1e-3, q_bias=1e-5, r_meas=0.03):
        self.x = np.zeros(2)            # [angle, bias]
        self.P = np.eye(2) * 1e-2
        self.Q = np.diag([q_angle, q_bias])
        self.R = r_meas

    def update(self, omega, theta_meas, dt):
        # --- Predict ---
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        B = np.array([dt, 0.0])
        self.x = F @ self.x + B * omega
        self.P = F @ self.P @ F.T + self.Q
        # --- Update (H = [1, 0]) ---
        S = self.P[0, 0] + self.R
        K = self.P[:, 0] / S
        y = theta_meas - self.x[0]
        self.x = self.x + K * y
        self.P = (np.eye(2) - np.outer(K, [1.0, 0.0])) @ self.P
        return self.x[0]


class KalmanEstimator:
    """Wrap two KalmanAxis filters: one for roll, one for pitch."""
    def __init__(self, **kw):
        self.kr = KalmanAxis(**kw)
        self.kp = KalmanAxis(**kw)

    def update(self, acc, gyro, dt):
        ax, ay, az = acc
        gx, gy, gz = gyro
        roll_meas = math.atan2(ay, az)
        pitch_meas = math.atan2(-ax, math.sqrt(ay * ay + az * az))
        roll = self.kr.update(gx, roll_meas, dt)
        pitch = self.kp.update(gy, pitch_meas, dt)
        return roll, pitch


# ===========================================================================
#  Algorithm 3: Madgwick filter (quaternion gradient descent, 6-axis form)
# ===========================================================================
class MadgwickEstimator:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion

    def update(self, acc, gyro, dt):
        q1, q2, q3, q4 = self.q  # [w, x, y, z]
        gx, gy, gz = gyro

        # Rate of change from gyroscope
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)

        ax, ay, az = acc
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 0.0:
            ax, ay, az = ax / norm, ay / norm, az / norm
            # Auxiliary variables
            _2q1 = 2.0 * q1
            _2q2 = 2.0 * q2
            _2q3 = 2.0 * q3
            _2q4 = 2.0 * q4
            _4q1 = 4.0 * q1
            _4q2 = 4.0 * q2
            _4q3 = 4.0 * q3
            _8q2 = 8.0 * q2
            _8q3 = 8.0 * q3
            q1q1 = q1 * q1
            q2q2 = q2 * q2
            q3q3 = q3 * q3
            q4q4 = q4 * q4
            # Gradient (objective function f for accelerometer)
            s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay
            s2 = (_4q2 * q4q4 - _2q4 * ax + 4.0 * q1q1 * q2 - _2q1 * ay
                  - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az)
            s3 = (4.0 * q1q1 * q3 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay
                  - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az)
            s4 = 4.0 * q2q2 * q4 - _2q2 * ax + 4.0 * q3q3 * q4 - _2q3 * ay
            n = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
            if n > 0.0:
                s1, s2, s3, s4 = s1 / n, s2 / n, s3 / n, s4 / n
                qDot1 -= self.beta * s1
                qDot2 -= self.beta * s2
                qDot3 -= self.beta * s3
                qDot4 -= self.beta * s4

        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        n = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = np.array([q1 / n, q2 / n, q3 / n, q4 / n])
        return quat_to_euler(self.q)


# ===========================================================================
#  Algorithm 4: Mahony filter (quaternion PI complementary, 6-axis form)
# ===========================================================================
class MahonyEstimator:
    def __init__(self, kp=10.0, ki=0.0):
        self.kp = kp
        self.ki = ki
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.eInt = np.zeros(3)

    def update(self, acc, gyro, dt):
        q1, q2, q3, q4 = self.q
        gx, gy, gz = gyro

        ax, ay, az = acc
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 0.0:
            ax, ay, az = ax / norm, ay / norm, az / norm
            # Predicted gravity direction in body frame
            vx = 2.0 * (q2 * q4 - q1 * q3)
            vy = 2.0 * (q1 * q2 + q3 * q4)
            vz = q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4
            # Error = measured x predicted (cross product)
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx
            # Integral term
            if self.ki > 0.0:
                self.eInt += np.array([ex, ey, ez]) * dt
                gx += self.ki * self.eInt[0]
                gy += self.ki * self.eInt[1]
                gz += self.ki * self.eInt[2]
            # Proportional term
            gx += self.kp * ex
            gy += self.kp * ey
            gz += self.kp * ez

        # Quaternion derivative
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz)
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy)
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx)
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx)

        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        n = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = np.array([q1 / n, q2 / n, q3 / n, q4 / n])
        return quat_to_euler(self.q)


# ---------------------------------------------------------------------------
#  Convenience: run any estimator over a (acc, gyro) array
# ---------------------------------------------------------------------------
def run_estimator(est, acc, gyro, fs):
    n = acc.shape[0]
    out = np.zeros((n, 2))
    dt = 1.0 / fs
    for i in range(n):
        if isinstance(est, TrigEstimator):
            r, p = est.update(acc[i])
        else:
            r, p = est.update(acc[i], gyro[i], dt)
        out[i, 0] = r
        out[i, 1] = p
    return out


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # Smoke test: run each estimator on a few samples
    rng = np.random.default_rng(0)
    acc = np.array([0.0, 9.81 * math.sin(math.radians(15)),
                    9.81 * math.cos(math.radians(15))])
    gyro = np.array([0.0, 0.0, 0.0])
    print("Static acc =", acc, "expected roll = 15 deg")
    for est in [TrigEstimator(), KalmanEstimator(), MadgwickEstimator(),
                MahonyEstimator()]:
        # Warm up the filters with a few samples for stateful ones
        for _ in range(500):
            if isinstance(est, TrigEstimator):
                r, p = est.update(acc)
            else:
                r, p = est.update(acc, gyro, 0.01)
        print(f"{est.__class__.__name__:18s} -> "
              f"roll={math.degrees(r):+6.2f} deg, pitch={math.degrees(p):+6.2f} deg")
