import numpy as np


class KalmanTracker:
    def __init__(self,
                 dt: float = 0.2,
                 process_noise_pos: float = 0.005,
                 process_noise_vel: float = 0.05,
                 measurement_noise: float = 0.02):
        self.dt = dt
        self.initialized = False

        self.x = np.zeros(6)          # state: [x,y,z,vx,vy,vz]
        self.P = np.eye(6) * 1.0      # state covariance

        # Constant-velocity transition: x_k = F x_{k-1}
        self.F = np.eye(6)
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        # Observe position only
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.Q = np.diag([process_noise_pos] * 3 + [process_noise_vel] * 3)
        self.R_base = np.eye(3) * measurement_noise

    def initialize(self, position: np.ndarray) -> None:
        """Seed the filter with a known position; velocity starts at zero."""
        self.x[:3] = position.copy()
        self.x[3:] = 0.0
        self.P = np.eye(6) * 1.0
        self.initialized = True

    def predict(self) -> np.ndarray:
        """Propagate state one timestep forward. Returns predicted position."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3].copy()

    def update(self, measurement: np.ndarray,
               occlusion_ratio: float = 0.0,
               occlusion_r_scale: float = 5.0) -> np.ndarray:
        R = self.R_base * (1.0 + occlusion_r_scale * occlusion_ratio)

        y = measurement - self.H @ self.x          # innovation
        S = self.H @ self.P @ self.H.T + R         # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)   # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        return self.x[:3].copy()

    def get_position(self) -> np.ndarray:
        return self.x[:3].copy()

    def get_velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    def zero_velocity(self) -> None:
        self.x[3:] = 0.0

    def reset(self) -> None:
        self.initialized = False
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1.0
