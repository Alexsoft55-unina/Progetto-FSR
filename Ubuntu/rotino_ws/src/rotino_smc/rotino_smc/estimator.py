"""
State estimator of the WBR controller (Cui et al., Micromachines 2022, 13, 747), NumPy only:
  - AxisKalman:        linear Kalman filter of eqs. 20-21, one instance per world axis

The LQR and MPC blocks that used to live here were removed when the balance law became a pure
sliding-mode cascade; the sliding surfaces are built directly on wbr_model.vlwip_coefficients.
"""

import numpy as np


class AxisKalman:
    """Eqs. (20)-(21) for one world axis: state [p, v] of the torso, input IMU acceleration,
    observation [p, v] from wheel odometry + leg kinematics."""

    def __init__(self, q_acc, r_pos, r_vel):
        self.q_acc = q_acc
        self.R_nom = np.diag([r_pos, r_vel])
        self.x = np.zeros(2)
        self.P = np.eye(2) * 1e-4

    def reset(self, p, v):
        self.x = np.array([p, v], dtype=float)
        self.P = np.eye(2) * 1e-4

    def predict(self, acc, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Bu = np.array([0.5 * dt * dt, dt])
        self.x = F @ self.x + Bu * acc
        self.P = F @ self.P @ F.T + self.q_acc * np.outer(Bu, Bu) + 1e-9 * np.eye(2)

    def correct(self, p_obs, v_obs, noise_scale=1.0):
        R = self.R_nom * noise_scale
        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ (np.array([p_obs, v_obs]) - self.x)
        self.P = (np.eye(2) - K) @ self.P
