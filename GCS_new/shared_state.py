# -*- coding: utf-8 -*-
"""
sik.py/lora.py 두 파이프라인 스레드가 공유해야 하는 작은 스레드-세이프
상태 객체들 - 전에는 geo_origin.py(SharedOrigin)와 flight_log.py
(FlightLogDir/RotatingCsvState/EventLogger)로 나뉘어 있었는데, 전부 같은
성격(main.py가 하나 만들어서 양쪽 파이프라인에 주입하는 조율용 객체)이라
파일 하나로 합쳤다.

- SharedOrigin: SiK/LoRa가 같은 ENU 원점(lat0/lon0)을 쓰게 조율
- FlightLogDir / RotatingCsvState: 비행별 로그 디렉터리 전환 + CSV 파일 교체
- EventLogger: 상태전이/메시지/명령 등 저빈도 이벤트를 JSONL로 기록
"""

import json
import math
import os
import threading
import time

_EARTH_METERS_PER_DEG = 111320.0


class SharedOrigin:
    """
    SiK/LoRa 파이프라인이 같은 ENU 원점(lat0/lon0)을 공유하기 위한 객체.

    각자 독립적으로 "첫 유효 fix"를 원점으로 잡으면, 연결 시점이 다를 때
    (예: SiK가 실내에서 먼저 연결되고 LoRa는 발사대에서 나중에 연결) 두
    원점이 실제로 수십~수백 미터 떨어진 서로 다른 기준점이 되어 3D 뷰에서
    두 궤적이 전혀 다른 좌표에 그려지는 문제가 있었다. main.py가 이 객체를
    하나만 만들어 양쪽에 주입하면, 두 스레드 중 어느 쪽이든 먼저 유효한
    fix를 받는 쪽으로 원점이 확정되고 나머지 한쪽은 그 값을 그대로 따라 쓴다.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._lat0 = None
        self._lon0 = None
        self._meters_per_deg_lon = None

    def set_if_needed(self, lat: float, lon: float) -> None:
        with self._lock:
            if self._lat0 is None:
                self._lat0 = lat
                self._lon0 = lon
                self._meters_per_deg_lon = _EARTH_METERS_PER_DEG * math.cos(math.radians(lat))
                print(f"[shared_state] 공유 ENU 원점 확정: lat={lat:.7f}, lon={lon:.7f}")

    def is_set(self) -> bool:
        with self._lock:
            return self._lat0 is not None

    def get_origin(self):
        """(lat0, lon0) - 아직 원점이 안 잡혔으면 (None, None)."""
        with self._lock:
            return self._lat0, self._lon0

    def to_en(self, lat: float, lon: float):
        """원점 기준 (pos_e, pos_n) 미터. 원점이 아직 없으면 (0.0, 0.0)."""
        with self._lock:
            if self._lat0 is None:
                return 0.0, 0.0
            pos_n = (lat - self._lat0) * _EARTH_METERS_PER_DEG
            pos_e = (lon - self._lon0) * self._meters_per_deg_lon
            return pos_e, pos_n


class FlightLogDir:
    """
    main.py가 "발사 인가"/FC Reboot/ASCENT 전이 시점에 새 디렉터리를
    요청하면, sik.py/lora.py 파이프라인 스레드가 (연결을 안 끊고) 다음
    루프에서 그 디렉터리로 CSV 파일을 새로 열게 하는 스레드 간 신호 채널.
    generation 카운터로 "마지막으로 내가 반영한 디렉터리인지"를 비교한다.
    """

    def __init__(self, initial_dir):
        self._lock = threading.Lock()
        self._dir = initial_dir
        self._generation = 0

    def current(self):
        """(디렉터리 경로, generation) - generation이 마지막으로 본 값과 다르면 새로 연 것."""
        with self._lock:
            return self._dir, self._generation

    def request_new_dir(self, new_dir):
        with self._lock:
            self._dir = new_dir
            self._generation += 1


class RotatingCsvState:
    """
    sik.py/lora.py가 각자 거의 똑같이 들고 있던 _CsvState + 로테이션 체크
    코드를 하나로 합친 것 - "메인 CSV + 통계 CSV, 파일 4개를 세대(generation)
    맞춰 통째로 갈아끼운다"는 동작 자체는 두 파이프라인이 완전히 동일해서,
    각자 파일 스키마(_ensure_csv_writer/_ensure_stats_csv_writer)만
    주입받게 분리했다.
    """

    def __init__(self, cfg, ensure_csv_writer, ensure_stats_csv_writer, flight_log_dir=None, log_prefix=""):
        self._cfg = cfg
        self._ensure_csv_writer = ensure_csv_writer
        self._ensure_stats_csv_writer = ensure_stats_csv_writer
        self._flight_log_dir = flight_log_dir
        self._log_prefix = log_prefix

        initial_dir, initial_gen = flight_log_dir.current() if flight_log_dir is not None else (cfg.CSV_OUTPUT_DIR, 0)
        self.csv_file, self.csv_writer = ensure_csv_writer(cfg, initial_dir)
        self.stats_csv_file, self.stats_csv_writer = ensure_stats_csv_writer(cfg, initial_dir)
        self.dir_generation = initial_gen

    def rotate_if_needed(self) -> bool:
        """세대가 바뀌었으면 파일을 새 디렉터리 것으로 교체하고 True를 반환한다.
        (반환값으로 호출자가 자기 쪽 flush 타이머 등을 리셋할지 판단한다.)"""
        if self._flight_log_dir is None:
            return False
        new_dir, new_gen = self._flight_log_dir.current()
        if new_gen == self.dir_generation:
            return False

        self.csv_file.close()
        self.stats_csv_file.close()
        self.csv_file, self.csv_writer = self._ensure_csv_writer(self._cfg, new_dir)
        self.stats_csv_file, self.stats_csv_writer = self._ensure_stats_csv_writer(self._cfg, new_dir)
        self.dir_generation = new_gen
        print(f"[{self._log_prefix}] 새 비행 로그 디렉터리로 전환: {new_dir}")
        return True

    def close(self):
        self.csv_file.close()
        self.stats_csv_file.close()


class EventLogger:
    """
    main.py(GUI 스레드)에서 상태 전이/메시지 로그/서보 명령/링크 전환/리부트
    이벤트처럼 저빈도 이벤트를 하나의 JSONL(줄마다 JSON 객체 하나) 파일에
    모아 기록한다. 이벤트마다 종류가 달라 CSV처럼 고정 컬럼을 쓰기보다
    JSONL이 다루기 쉽다. FlightLogDir이 새 디렉터리를 요청하면 다음 log()
    호출 때 그 디렉터리의 새 파일로 넘어간다.
    """

    def __init__(self, flight_log_dir, filename="events.jsonl", flush_interval_sec=1.0):
        self._flight_log_dir = flight_log_dir
        self._filename = filename
        self._file = None
        self._dir_generation = None
        # 이벤트마다 flush()하면(메시지 로그는 STATUSTEXT가 몰릴 때 꽤
        # 잦을 수 있다) sik.py/lora.py CSV에서 겪었던 것과 같은 이유로
        # GUI 스레드가 디스크/동기화 I/O에 잠깐씩 막힐 수 있다 - 그래서
        # write()는 매번 하되 flush()는 이 간격으로만 한다.
        self._flush_interval_sec = flush_interval_sec
        self._last_flush = 0.0
        self._reopen_if_needed()

    def _reopen_if_needed(self):
        current_dir, generation = self._flight_log_dir.current()
        if generation == self._dir_generation:
            return
        if self._file is not None:
            self._file.close()
        os.makedirs(current_dir, exist_ok=True)
        self._file = open(os.path.join(current_dir, self._filename), mode="a", encoding="utf-8")
        self._dir_generation = generation
        self._last_flush = 0.0

    def log(self, event_type: str, **fields) -> None:
        self._reopen_if_needed()
        record = {"ts": time.time(), "type": event_type, **fields}
        self._file.write(json.dumps(record, ensure_ascii=False) + "\n")
        now = time.time()
        if now - self._last_flush >= self._flush_interval_sec:
            self._file.flush()
            self._last_flush = now
