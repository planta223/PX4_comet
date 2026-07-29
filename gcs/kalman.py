# -*- coding: utf-8 -*-
"""
6D 선형 칼만필터 (SiK 전용).

상태 벡터 x = [pos_e, pos_n, alt, vel_e, vel_n, vel_z]^T
입력 u = accel = [accel_e, accel_n, accel_z]^T (raw IMU에서 변환된 가속도)

관측은 두 종류를 독립적으로 받는다 (E/N/Z축이 F/B 행렬상 서로 완전히
분리되어 있어 수학적으로 문제없다):
  - 수평 관측 z_h = [pos_e, pos_n]^T (GPS, ~5Hz)
  - 수직 관측 z_v = [alt]^T (기압계, ~10Hz - GPS보다 빠르고 정확)
각자 도착하는 대로 update_horizontal()/update_vertical()을 호출한다.
GPS 5Hz에 기압계를 묶어서 같이 기다리게 하면 기압계의 빠른 갱신률을
버리게 되므로 일부러 따로 둔다.

자세(attitude)는 다루지 않는다 - 온보드 EKF3가 별도로 처리한다.
"""

import numpy as np

import config


class KalmanFilter6D:
    def __init__(self):
        self.x = np.zeros((6, 1), dtype=float)
        self.P = config.KF_INITIAL_COVARIANCE.copy()
        self.Q = config.KF_PROCESS_NOISE.copy()
        self.R_horizontal = config.KF_OBSERVATION_NOISE_HORIZONTAL.copy()
        self.R_vertical = config.KF_OBSERVATION_NOISE_VERTICAL.copy()

        # 관측 모델: GPS는 수평 위치(pos_e, pos_n)만 관측
        self.H_horizontal = np.zeros((2, 6), dtype=float)
        self.H_horizontal[0, 0] = 1.0
        self.H_horizontal[1, 1] = 1.0

        # 관측 모델: 기압계는 고도(alt)만 관측
        self.H_vertical = np.zeros((1, 6), dtype=float)
        self.H_vertical[0, 2] = 1.0

    def predict(self, dt: float, accel: np.ndarray) -> None:
        """
        dt: 이전 predict/update 이후 경과 시간 (초)
        accel: shape (3,) 또는 (3,1), [accel_e, accel_n, accel_z] (m/s^2)
        """
        accel = np.asarray(accel, dtype=float).reshape(3, 1)

        F = np.eye(6, dtype=float)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        B = np.zeros((6, 3), dtype=float)
        half_dt2 = 0.5 * dt * dt
        B[0, 0] = half_dt2
        B[1, 1] = half_dt2
        B[2, 2] = half_dt2
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt

        self.x = F @ self.x + B @ accel
        self.P = F @ self.P @ F.T + self.Q

    def _update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6, dtype=float) - K @ H) @ self.P

    def update_horizontal(self, gps_xy: np.ndarray) -> None:
        """
        gps_xy: shape (2,) 또는 (2,1), [pos_e, pos_n] (m) - GPS 도착 시 호출.
        """
        z = np.asarray(gps_xy, dtype=float).reshape(2, 1)
        self._update(z, self.H_horizontal, self.R_horizontal)

    def update_vertical(self, baro_alt: float) -> None:
        """
        baro_alt: 기압계 기반 상대 고도(m) - 기압계 메시지 도착 시 호출.
        """
        z = np.asarray([[baro_alt]], dtype=float)
        self._update(z, self.H_vertical, self.R_vertical)

    def get_state(self) -> dict:
        s = self.x.flatten()
        return {
            "pos_e": float(s[0]),
            "pos_n": float(s[1]),
            "alt": float(s[2]),
            "vel_e": float(s[3]),
            "vel_n": float(s[4]),
            "vel_z": float(s[5]),
        }
