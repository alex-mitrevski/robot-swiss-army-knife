"""Quaternion EKF for object orientation and angular velocity.

State: x = [q0, q1, q2, q3, wx, wy, wz]  (scalar-first quaternion + camera-frame ω)
Motion model: constant angular velocity  →  q_{k+1} = q_k ⊗ exp(ω_k · dt/2)
Measurement: quaternion extracted from the raw PCA rotation matrix.
"""

import numpy as np

def _qmul(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def _qleft(q):
    """Q_L(q): 4x4 matrix s.t. q⊗p = Q_L(q)·p  for any p."""
    w, x, y, z = q
    return np.array([
        [ w, -x, -y, -z],
        [ x,  w, -z,  y],
        [ y,  z,  w, -x],
        [ z, -y,  x,  w],
    ])


def _qright(q):
    """Q_R(q): 4x4 matrix s.t. p⊗q = Q_R(q)·p  for any p."""
    w, x, y, z = q
    return np.array([
        [ w, -x, -y, -z],
        [ x,  w,  z, -y],
        [ y, -z,  w,  x],
        [ z,  y, -x,  w],
    ])


def _R_to_quat(R):
    """3x3 rotation matrix → unit quaternion [w,x,y,z]"""
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        return np.array([0.25/s,
                         (R[2,1]-R[1,2])*s,
                         (R[0,2]-R[2,0])*s,
                         (R[1,0]-R[0,1])*s])
    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([(R[2,1]-R[1,2])/s, 0.25*s,
                         (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
    if R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s,
                         0.25*s, (R[1,2]+R[2,1])/s])
    s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
    return np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s,
                     (R[1,2]+R[2,1])/s, 0.25*s])


def _quat_to_R(q):
    """Unit quaternion [w,x,y,z] → 3x3 rotation matrix (rows = principal axes)."""
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),    2*(x*z+w*y)  ],
        [2*(x*y+w*z),    1-2*(x*x+z*z),  2*(y*z-w*x)  ],
        [2*(x*z-w*y),    2*(y*z+w*x),    1-2*(x*x+y*y)],
    ])

class RotationTracker:

    def __init__(self, dt,
                 process_noise_q=1e-4,
                 process_noise_w=0.01,
                 measurement_noise=0.01):
        self._dt = dt
        self._Q  = np.diag([process_noise_q]*4 + [process_noise_w]*3)
        self._R  = np.eye(4) * measurement_noise
        # H selects the quaternion part of the state (linear measurement model)
        self._H  = np.hstack([np.eye(4), np.zeros((4, 3))])
        self._x  = None   # [q0,q1,q2,q3,wx,wy,wz]
        self._P  = None   # 7×7 covariance
        self.initialized = False

    def initialize(self, R_mat):
        """Seed tracker from a 3×3 rotation matrix (rows = principal axes)."""
        q = _R_to_quat(R_mat)
        q /= np.linalg.norm(q)
        self._x = np.concatenate([q, np.zeros(3)])
        self._P = np.eye(7) * 0.1
        self.initialized = True

    def predict(self):
        """EKF predict step.

        Returns the predicted rotation matrix so the caller can sign-align
        the incoming PCA measurement before calling update().
        """
        q, w = self._x[:4], self._x[4:]
        dt   = self._dt

        # Integrate: q_new = q ⊗ exp(ω·dt/2)
        angle = np.linalg.norm(w) * dt
        if angle > 1e-9:
            axis = w / np.linalg.norm(w)
            dq   = np.concatenate([[np.cos(angle/2)], np.sin(angle/2)*axis])
        else:
            dq = np.array([1.0, 0.0, 0.0, 0.0])

        q_new = _qmul(q, dq)
        q_new /= np.linalg.norm(q_new)
        self._x[:4] = q_new

        # Jacobian F (7×7): ∂q_new/∂q = Q_R(dq),  ∂q_new/∂ω = dt/2·Q_L(q)[:,1:]
        F          = np.eye(7)
        F[:4, :4]  = _qright(dq)
        F[:4, 4:]  = (dt / 2.0) * _qleft(q)[:, 1:]

        self._P = F @ self._P @ F.T + self._Q
        return _quat_to_R(q_new)

    def update(self, R_mat, occlusion_ratio=0.0, occlusion_r_scale=1.0):
        q_meas = _R_to_quat(R_mat)
        q_meas /= np.linalg.norm(q_meas)

        if np.dot(q_meas, self._x[:4]) < 0:
            q_meas = -q_meas

        R_noise = self._R * (1.0 + occlusion_ratio * (occlusion_r_scale - 1.0))
        S = self._H @ self._P @ self._H.T + R_noise
        K = self._P @ self._H.T @ np.linalg.inv(S)

        innov    = q_meas - self._H @ self._x
        self._x  = self._x + K @ innov
        self._x[:4] /= np.linalg.norm(self._x[:4])
        self._P = (np.eye(7) - K @ self._H) @ self._P

    def zero_angular_velocity(self):
        """Zero ω — call on full-OCC entry when |ω| < ANGULAR_VELOCITY_ZERO_THRESH."""
        self._x[4:] = 0.0

    def get_rotation(self):
        """Best-estimate 3x3 rotation matrix (rows = principal axes)."""
        return _quat_to_R(self._x[:4])

    def get_angular_velocity(self):
        """Best-estimate angular velocity (rad/s, camera frame)."""
        return self._x[4:].copy()

    def get_quaternion(self):
        return self._x[:4].copy()
