# -*- coding: utf-8 -*-
"""
패킷 품질/위치·속도 안정성을 정량화하기 위한 공용 통계 유틸리티.

순수 계산 클래스만 담는다 - 시리얼/큐/스레드 등 어떤 것에도 의존하지
않는다 (sik.py/lora.py/main.py가 각자 인스턴스를 만들어 자기 데이터를
먹인다).
"""

import math
from collections import deque


class RateStats:
    """도착 타임스탬프로 실측 갱신률(Hz)과 지터(도착 간격의 표준편차, ms)를 낸다."""

    def __init__(self, window=50):
        self._timestamps = deque(maxlen=window)

    def record(self, now: float) -> None:
        self._timestamps.append(now)

    def rate_hz(self) -> float:
        if len(self._timestamps) < 2:
            return 0.0
        span = self._timestamps[-1] - self._timestamps[0]
        if span <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / span

    def jitter_ms(self) -> float:
        if len(self._timestamps) < 3:
            return 0.0
        ts = list(self._timestamps)
        intervals = [b - a for a, b in zip(ts, ts[1:])]
        mean = sum(intervals) / len(intervals)
        variance = sum((x - mean) ** 2 for x in intervals) / len(intervals)
        return math.sqrt(variance) * 1000.0


class SeqLossTracker:
    """
    0~255 wraparound 시퀀스 번호를 보고 손실률을 추정한다 (세션 누적치 -
    RateStats처럼 이동창이 아니라, 연결된 이후 전체 기준 손실률).
    """

    def __init__(self):
        self.received = 0
        self.lost = 0
        self._last_seq = None

    def record(self, seq: int) -> None:
        self.received += 1
        if self._last_seq is not None:
            expected = (self._last_seq + 1) % 256
            if seq != expected:
                gap = (seq - expected) % 256
                self.lost += gap
        self._last_seq = seq

    def loss_pct(self) -> float:
        total = self.received + self.lost
        if total <= 0:
            return 0.0
        return (self.lost / total) * 100.0


class Vec3RollingStats:
    """
    (e, n, z) 3축 값의 이동창 평균/표준편차 - 값이 얼마나 안정적인지
    (잡음 크기)를 정량화한다. 정지 상태에서 표준편차가 크게 나오면
    추정치가 흔들리고 있다는 뜻.
    """

    def __init__(self, window=100):
        self._window = window
        self._e = deque(maxlen=window)
        self._n = deque(maxlen=window)
        self._z = deque(maxlen=window)

    def record(self, vec3) -> None:
        e, n, z = vec3
        self._e.append(e)
        self._n.append(n)
        self._z.append(z)

    @staticmethod
    def _mean_std(values):
        if len(values) < 2:
            return 0.0, 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return mean, math.sqrt(variance)

    def mean(self) -> tuple:
        return (
            self._mean_std(self._e)[0],
            self._mean_std(self._n)[0],
            self._mean_std(self._z)[0],
        )

    def stddev(self) -> tuple:
        return (
            self._mean_std(self._e)[1],
            self._mean_std(self._n)[1],
            self._mean_std(self._z)[1],
        )
