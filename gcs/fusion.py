# -*- coding: utf-8 -*-
"""
SiK/LoRa 데이터 취사선택(판단) 라이브러리.

main.py가 매 프레임 호출해서, 필드별로 SiK/LoRa 중 어느 값을 GUI/기록에
올릴지 결정한다 - sik.py/lora.py는 이 판단을 하지 않고 받은 걸 무조건
큐/CSV로 흘려보내기만 하고(파이프라인), main.py는 여기서 나온 결과를
표시/필터링만 한다(소비자) - "판단"은 이 파일 하나로 모은다. 정책(고정):
  - 위치: 항상 LoRa 우선 (SiK 건강과 무관), LoRa 죽으면 SiK 자체 칼만필터 위치
  - 자세: SiK ATTITUDE만, 폴백 없음 (LoRa는 위치값만 보냄)
  - 가속도: SiK RAW_IMU만, 폴백 없음
  - 속도: SiK 자체 칼만필터, SiK 죽으면 LoRa 위치를 전용 칼만필터에 태워 추정
  - 로켓 상태: LoRa 텔레메트리 우선, LoRa 죽으면 SiK STATUSTEXT
    (예전엔 SiK 우선이었는데, 실제 비행 로그에서 SiK가 비행 3분 만에
    완전히 죽고 LoRa는 끊김을 반복하면서도 끝까지 재연결됐던 것을 보고
    우선순위를 뒤집었다 - "SiK가 더 안정적"이라는 가정이 실측과 반대였음)
  - 기록 가능 여부(recording_enabled): 위에서 정해진 로켓 상태가
    config.RECORDING_ENABLED_STATES에 속하는지로 판단 - main.py가 궤적
    트레일 등 "지금 이 데이터를 실제 비행 데이터로 취급할지"를 여기 하나만
    보고 결정하면 된다.

링크가 둘 다 죽어있으면 마지막 판단값을 그대로 유지한다(freeze) - 0으로
리셋하지 않는다 (마지막으로 알려진 상태가 더 유용한 정보이기 때문).
"""

import config
from kalman import KalmanFilter6D


def is_link_alive(last_seen, timeout_sec, now) -> bool:
    return last_seen is not None and (now - last_seen) <= timeout_sec


class DataFusion:
    def __init__(self):
        self._position = (0.0, 0.0, 0.0)
        self._position_source = None
        self._attitude = (0.0, 0.0, 0.0)  # radians
        self._attitude_source = None
        self._accel = None
        self._accel_source = None
        self._velocity = (0.0, 0.0, 0.0)
        self._velocity_source = None
        self._rocket_state = "READY"
        self._rocket_state_source = None

        # SiK가 죽었을 때 LoRa 위치를 관측으로 흘려보내 속도를 추정하는
        # 전용 칼만필터 - sik.py 안의 kf와는 완전히 별개 인스턴스이며,
        # SiK가 살아있는 동안은 predict/update를 아예 돌리지 않는다(대기만).
        self._fallback_kf = KalmanFilter6D()
        self._fallback_kf_predict_time = None
        self._fallback_kf_last_sample_ts = None

    def _update_velocity(self, now, sik_alive, sik_vel, lora_pos_sample):
        if sik_alive:
            # SiK가 다시 살아났으면, 다음 폴백 진입 시 오래된 dt로 튀지
            # 않도록 predict 타이머만 리셋한다 (필터 상태 자체는 유지해도 됨).
            self._fallback_kf_predict_time = None
            if sik_vel is not None:
                self._velocity = sik_vel
                self._velocity_source = "kf"
            return

        dt = 0.0 if self._fallback_kf_predict_time is None else (now - self._fallback_kf_predict_time)
        self._fallback_kf_predict_time = now
        if dt > 0:
            self._fallback_kf.predict(dt, (0.0, 0.0, 0.0))

        if lora_pos_sample is not None:
            sample_ts = lora_pos_sample["timestamp"]
            if sample_ts != self._fallback_kf_last_sample_ts:
                self._fallback_kf.update_horizontal((lora_pos_sample["pos_e"], lora_pos_sample["pos_n"]))
                self._fallback_kf.update_vertical(lora_pos_sample["alt"])
                self._fallback_kf_last_sample_ts = sample_ts

        state = self._fallback_kf.get_state()
        self._velocity = (state["vel_e"], state["vel_n"], state["vel_z"])
        self._velocity_source = "kf_fallback"

    def update(
        self, now, sik_alive, lora_alive,
        sik_pos, sik_attitude_rad, sik_accel, sik_vel, sik_rocket_state,
        lora_pos_sample, lora_rocket_state,
    ) -> dict:
        # 위치: 항상 LoRa 우선, LoRa 없으면 SiK
        if lora_alive and lora_pos_sample is not None:
            self._position = (lora_pos_sample["pos_e"], lora_pos_sample["pos_n"], lora_pos_sample["alt"])
            self._position_source = "lora"
        elif sik_alive and sik_pos is not None:
            self._position = sik_pos
            self._position_source = "sik"
        # 둘 다 없으면 마지막 값 유지

        # 자세: SiK 전용, 폴백 없음
        if sik_alive and sik_attitude_rad is not None:
            self._attitude = sik_attitude_rad
            self._attitude_source = "sik"
        elif not sik_alive:
            self._attitude_source = None

        # 가속도: SiK 전용, 폴백 없음
        if sik_alive and sik_accel is not None:
            self._accel = sik_accel
            self._accel_source = "sik"
        elif not sik_alive:
            self._accel = None
            self._accel_source = None

        # 속도: SiK 칼만필터, SiK 죽으면 LoRa 위치를 별도 칼만필터에 태움
        self._update_velocity(now, sik_alive, sik_vel, lora_pos_sample)

        # 로켓 상태: LoRa 우선, LoRa 죽으면 SiK (위 모듈 docstring 참고)
        if lora_alive and lora_rocket_state is not None:
            self._rocket_state = lora_rocket_state
            self._rocket_state_source = "lora"
        elif sik_alive and sik_rocket_state is not None:
            self._rocket_state = sik_rocket_state
            self._rocket_state_source = "sik"

        return {
            "position": self._position,
            "position_source": self._position_source,
            "attitude": self._attitude,
            "attitude_source": self._attitude_source,
            "accel": self._accel,
            "accel_source": self._accel_source,
            "velocity": self._velocity,
            "velocity_source": self._velocity_source,
            "rocket_state": self._rocket_state,
            "rocket_state_source": self._rocket_state_source,
            "recording_enabled": self._rocket_state in config.RECORDING_ENABLED_STATES,
        }
