# -*- coding: utf-8 -*-
"""
3D 궤적/로켓 와이어프레임을 위한 순수 수학 유틸리티.

Dear PyGui에 의존하지 않는다. 여기서 계산한 2D 화면 좌표를 main.py가
dpg.draw_line/draw_polyline로 그리기만 한다 (GUI 코드에 좌표계산/회전
로직을 넣지 않기 위한 분리).

월드 좌표계: X = East, Y = Up, Z = North (미터).
로켓 body frame: X = 전방(노즈 방향), Y = 우, Z = 하 (표준 항공기 body 축).
"""

import math

import numpy as np


def orbit_eye_position(target, azimuth_rad: float, elevation_rad: float, distance: float) -> np.ndarray:
    """target을 중심으로 구면좌표(azimuth, elevation, distance)에 있는 카메라 위치."""
    cx, cy, cz = target
    x = cx + distance * math.cos(elevation_rad) * math.sin(azimuth_rad)
    y = cy + distance * math.sin(elevation_rad)
    z = cz + distance * math.cos(elevation_rad) * math.cos(azimuth_rad)
    return np.array([x, y, z], dtype=float)


class Camera:
    """단순 핀홀 카메라 - 월드 좌표를 화면 픽셀좌표로 투영한다."""

    def __init__(self, eye, target, up, fov_rad: float, aspect: float, near: float, far: float):
        self.eye = np.asarray(eye, dtype=float)
        self.target = np.asarray(target, dtype=float)
        self.near = near
        self.far = far
        self.aspect = aspect if aspect > 1e-6 else 1.0

        forward = self.target - self.eye
        norm = np.linalg.norm(forward)
        forward = forward / norm if norm > 1e-9 else np.array([0.0, 0.0, 1.0])

        up_vec = np.asarray(up, dtype=float)
        right = np.cross(up_vec, forward)
        rnorm = np.linalg.norm(right)
        if rnorm < 1e-9:
            # forward가 up과 거의 평행한 극단적인 경우 - 임의의 축으로 대체.
            right = np.array([1.0, 0.0, 0.0])
        else:
            right = right / rnorm
        true_up = np.cross(forward, right)

        self._forward = forward
        self._right = right
        self._up = true_up
        self._focal = 1.0 / math.tan(fov_rad / 2.0)

    def project(self, point_world):
        """
        point_world -> 정규화 화면좌표 (ndc_x, ndc_y) (-1..1 근방, 화면 밖도 가능).
        카메라 뒤쪽/near보다 가까운 점은 None (클리핑).
        """
        rel = np.asarray(point_world, dtype=float) - self.eye
        view_z = float(np.dot(rel, self._forward))
        if view_z <= self.near:
            return None

        view_x = float(np.dot(rel, self._right))
        view_y = float(np.dot(rel, self._up))

        ndc_x = (view_x / view_z) * self._focal / self.aspect
        ndc_y = (view_y / view_z) * self._focal
        return ndc_x, ndc_y

    def project_to_screen(self, point_world, width: float, height: float):
        """project()의 NDC 좌표를 실제 픽셀좌표 (px, py)로 변환. 클리핑되면 None."""
        ndc = self.project(point_world)
        if ndc is None:
            return None
        ndc_x, ndc_y = ndc
        screen_x = (ndc_x + 1.0) * 0.5 * width
        screen_y = (1.0 - (ndc_y + 1.0) * 0.5) * height
        return screen_x, screen_y


def body_to_world(local_point, roll: float, pitch: float, yaw: float,
                   origin_e: float, origin_n: float, origin_alt: float):
    """
    로켓 body frame(X=전방, Y=우, Z=하)의 한 점을 mavlink ATTITUDE의
    roll/pitch/yaw(rad, 표준 body->NED 회전 R = Rz(yaw)*Ry(pitch)*Rx(roll))로
    회전시킨 뒤 현재 위치만큼 평행이동해 월드 좌표(East, Up, North)로 변환한다.
    """
    x, y, z = local_point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr

    north = r00 * x + r01 * y + r02 * z
    east = r10 * x + r11 * y + r12 * z
    down = r20 * x + r21 * y + r22 * z

    world_e = east + origin_e
    world_up = -down + origin_alt
    world_n = north + origin_n
    return world_e, world_up, world_n


def build_rocket_wireframe(length_m: float, diameter_m: float, fin_span_m: float, nose_ratio: float):
    """
    body frame(X=전방, Y=우, Z=하) 기준 로켓 와이어프레임 선분 목록.
    몸통 원통(사각 단면으로 단순화) + 노즈콘 + 핀 4개.

    반환: [(p_start, p_end), ...], 각 점은 (x, y, z) 튜플 (미터, body frame).
    """
    r = diameter_m / 2.0
    nose_len = length_m * nose_ratio
    nose_x = length_m / 2.0
    shoulder_x = nose_x - nose_len
    tail_x = -length_m / 2.0

    angles = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    tail_ring = [(tail_x, r * math.cos(a), r * math.sin(a)) for a in angles]
    shoulder_ring = [(shoulder_x, r * math.cos(a), r * math.sin(a)) for a in angles]
    fin_tips = [(tail_x, fin_span_m * math.cos(a), fin_span_m * math.sin(a)) for a in angles]
    nose_tip = (nose_x, 0.0, 0.0)

    segments = []
    for i in range(4):
        j = (i + 1) % 4
        segments.append((tail_ring[i], shoulder_ring[i]))      # 몸통 옆선
        segments.append((tail_ring[i], tail_ring[j]))          # 꼬리 링
        segments.append((shoulder_ring[i], shoulder_ring[j]))  # 어깨 링
        segments.append((shoulder_ring[i], nose_tip))          # 노즈콘
        segments.append((tail_ring[i], fin_tips[i]))           # 핀
    return segments
