# -*- coding: utf-8 -*-
"""
LoRa 텔레메트리 실측 갱신율 측정 도구.

comet_lora.lua가 실제로 보내는 텔레메트리 패킷(17바이트, header=0xA5,
CRC-8 poly=0x07/init=0x00)을 그대로 파싱해서, 실제 무선 링크에서 몇 Hz로
받히는지, 손실률은 얼마인지, 지터(도착 간격 편차)는 얼마인지를 실측한다.

주의: gcs/config.py의 LORA_PACKET_LENGTH(16)/LORA_CRC_POLYNOMIAL(CRC-16)은
comet_lora.lua의 실제 스펙(17바이트/CRC-8)과 다르므로 이 스크립트에서는
그 값들을 쓰지 않고 실제 스펙을 직접 정의한다.

사용법:
    python lora_rate_test.py --port COM7 --baud 57600
    (포트/보드레이트를 안 주면 config.py의 LORA_SERIAL_PORT/LORA_BAUDRATE 사용)

Ctrl+C로 종료하면 최종 통계를 출력한다.
"""

import argparse
import struct
import sys
import time

import serial

import config

TELEMETRY_HEADER = 0xA5
TELEMETRY_LENGTH = 17  # header+seq+state+fault_flags(4) + time_ms(4) + north/east(4) + alt(4)
                       # + crc8(1) = 17 (GCS는 위치값만 받음 - fusion.py 참고)
                       # north/east는 절대 위경도가 아니라 HOME 기준 오프셋
                       # (cm 단위, int16 -> ±327.67m, comet_sensor.lua 참고)

# header(B) seq(B) flight_state(B) fault_flags(B) time_ms(I) north(h) east(h) alt(i)
_STRUCT_FMT = "<BBBBIhhi"
assert struct.calcsize(_STRUCT_FMT) == TELEMETRY_LENGTH - 1, \
    "struct 포맷 길이가 CRC 제외 16바이트와 안 맞음 - _STRUCT_FMT 확인 필요"

FLIGHT_STATE_NAMES = ["READY", "ASCENT", "DEPLOY", "DESCENT", "LANDED"]

# comet_flight_state.lua의 M.FAULT 비트필드와 반드시 일치해야 한다
# (gcs/config.py의 LORA_FAULT_FLAG_NAMES와 동일).
FAULT_FLAG_NAMES = {
    0x01: "ACCEL_SENSOR_INVALID",
    0x02: "POSITION_INVALID",
    0x04: "VELOCITY_INVALID",
    0x08: "ASCENT_TIMEOUT",
    0x10: "DEPLOY_COMMAND_FAILED",
    0x20: "LORA_UNAVAILABLE",
}

# 비행 중(ASCENT/DEPLOY/DESCENT)엔 comet_config.lua의 FAST_PERIOD_MS(140ms,
# 목표 처리량 약 150B/s)를, 그 외엔 NORMAL_PERIOD_MS(1000ms)를 기대 주기로
# 삼아 실측과 비교한다.
_FAST_STATES = {1, 2, 3}
EXPECTED_FAST_PERIOD_S = 0.14
EXPECTED_NORMAL_PERIOD_S = 1.0


def calculate_crc8(data: bytes) -> int:
    """comet_lora.lua의 calculate_crc8()과 100% 동일한 알고리즘 (poly=0x07, init=0x00)."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class RateStats:
    """실행 중 누적 통계 + 주기적/최종 리포트."""

    def __init__(self):
        self.start_time = time.monotonic()
        self.valid_count = 0
        self.crc_fail_count = 0
        self.resync_byte_count = 0  # 헤더를 못 찾아 그냥 버린 바이트 수

        self.last_arrival_time = None
        self.inter_arrival_times = []

        self.last_sequence = None
        self.expected_lost = 0  # 시퀀스 갭으로 추정한 손실 패킷 수

        # 상태별(빠른 주기 기대 / 느린 주기 기대) 도착 간격을 따로 모은다.
        self.intervals_by_expected_period = {"fast": [], "normal": []}

    def record_crc_fail(self):
        self.crc_fail_count += 1

    def record_resync_byte(self):
        self.resync_byte_count += 1

    def record_valid_packet(self, fields: dict):
        now = time.monotonic()
        self.valid_count += 1

        if self.last_arrival_time is not None:
            dt = now - self.last_arrival_time
            self.inter_arrival_times.append(dt)

            bucket = "fast" if fields["flight_state"] in _FAST_STATES else "normal"
            self.intervals_by_expected_period[bucket].append(dt)

        self.last_arrival_time = now

        seq = fields["sequence"]
        if self.last_sequence is not None:
            expected_next = (self.last_sequence + 1) % 256
            if seq != expected_next:
                gap = (seq - expected_next) % 256
                self.expected_lost += gap
        self.last_sequence = seq

    def elapsed(self):
        return time.monotonic() - self.start_time

    def print_live_line(self, fields: dict):
        state_name = FLIGHT_STATE_NAMES[fields["flight_state"]] \
            if fields["flight_state"] < len(FLIGHT_STATE_NAMES) else "?"
        faults = [name for bit, name in FAULT_FLAG_NAMES.items() if fields["fault_flags"] & bit]
        fault_str = f" FAULT={'|'.join(faults)}" if faults else ""
        print(
            f"  [{self.elapsed():7.1f}s] seq={fields['sequence']:3d} "
            f"state={state_name:7s} north={fields['north_m']:.2f}m "
            f"east={fields['east_m']:.2f}m alt={fields['altitude']:.2f}m{fault_str} "
            f"(누적 수신 {self.valid_count}, 추정손실 {self.expected_lost}, "
            f"CRC실패 {self.crc_fail_count})"
        )

    def print_summary(self):
        elapsed = self.elapsed()
        total_expected = self.valid_count + self.expected_lost

        print("\n" + "=" * 60)
        print("LoRa 텔레메트리 갱신율 실측 결과")
        print("=" * 60)
        print(f"측정 시간           : {elapsed:.1f}s")
        print(f"유효 패킷 수신       : {self.valid_count}")
        print(f"CRC 검증 실패        : {self.crc_fail_count}")
        print(f"헤더 못 찾아 버린 바이트: {self.resync_byte_count}")
        print(f"시퀀스 갭 기반 추정 손실: {self.expected_lost} "
              f"(손실률 {100*self.expected_lost/total_expected:.2f}%)"
              if total_expected > 0 else "시퀀스 갭 기반 추정 손실: 계산 불가(수신 0건)")

        if self.valid_count > 1:
            measured_hz = self.valid_count / elapsed
            print(f"\n전체 평균 실측 갱신율 : {measured_hz:.2f} Hz "
                  f"(평균 주기 {1000/measured_hz:.0f} ms)")

        for bucket, expected_period_s, label in [
            ("fast", EXPECTED_FAST_PERIOD_S, "비행 중(ASCENT/DEPLOY/DESCENT) 기대 140ms"),
            ("normal", EXPECTED_NORMAL_PERIOD_S, "평시(READY/LANDED) 기대 1000ms"),
        ]:
            intervals = self.intervals_by_expected_period[bucket]
            if not intervals:
                continue
            avg = sum(intervals) / len(intervals)
            print(f"\n[{label}]")
            print(f"  샘플 수         : {len(intervals)}")
            print(f"  실측 평균 간격   : {avg*1000:.1f} ms (기대값 {expected_period_s*1000:.0f} ms)")
            print(f"  실측 최소/최대   : {min(intervals)*1000:.1f} / {max(intervals)*1000:.1f} ms")
        print("=" * 60)


def run(port: str, baud: int, duration: float = None):
    print(f"LoRa 포트 여는 중: {port} @ {baud} baud")
    ser = serial.Serial(port, baud, timeout=1.0)

    stats = RateStats()
    rx_buffer = bytearray()

    if duration is not None:
        print(f"수신 대기 중... ({duration:.0f}초 후 자동 종료)\n")
    else:
        print("수신 대기 중... (Ctrl+C로 종료하고 통계 출력)\n")

    try:
        while True:
            if duration is not None and stats.elapsed() >= duration:
                break

            byte = ser.read(1)
            if not byte:
                continue
            b = byte[0]

            if len(rx_buffer) == 0 and b != TELEMETRY_HEADER:
                stats.record_resync_byte()
                continue

            rx_buffer.append(b)

            if len(rx_buffer) < TELEMETRY_LENGTH:
                continue

            # 17바이트 다 모임 - CRC 검증
            payload = bytes(rx_buffer[:-1])
            received_crc = rx_buffer[-1]
            expected_crc = calculate_crc8(payload)

            if received_crc != expected_crc:
                stats.record_crc_fail()
                rx_buffer.clear()
                continue

            unpacked = struct.unpack(_STRUCT_FMT, payload)
            fields = {
                "header": unpacked[0],
                "sequence": unpacked[1],
                "flight_state": unpacked[2],
                "fault_flags": unpacked[3],
                "time_ms": unpacked[4],
                "north_m": unpacked[5] / 100.0,
                "east_m": unpacked[6] / 100.0,
                "altitude": unpacked[7] / 100.0,
            }

            stats.record_valid_packet(fields)
            stats.print_live_line(fields)

            rx_buffer.clear()

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        stats.print_summary()


def main():
    parser = argparse.ArgumentParser(description="LoRa 텔레메트리 갱신율 실측")
    parser.add_argument("--port", default=getattr(config, "LORA_SERIAL_PORT", None),
                         help="시리얼 포트 (기본값: config.LORA_SERIAL_PORT)")
    parser.add_argument("--baud", type=int,
                         default=getattr(config, "LORA_BAUDRATE", 57600),
                         help="baud rate (기본값: config.LORA_BAUDRATE)")
    parser.add_argument("--duration", type=float, default=None,
                         help="측정 시간(초). 안 주면 Ctrl+C로 종료할 때까지 계속 실행")
    args = parser.parse_args()

    if args.port is None:
        print("포트를 지정하세요 (--port COM7 등)")
        sys.exit(1)

    if args.port == getattr(config, "SIK_CONNECTION_STRING", None):
        print(f"경고: --port({args.port})가 config.SIK_CONNECTION_STRING과 동일합니다.")
        print("      SiK 라디오와 LoRa 모듈이 서로 다른 포트인지 확인하세요.\n")

    try:
        run(args.port, args.baud, args.duration)
    except serial.SerialException as exc:
        print(f"\n포트 열기/읽기 실패: {exc}")
        print("포트 이름(COM8 등)이 맞는지, 다른 프로그램(Mission Planner 등)이 "
              "이미 그 포트를 점유하고 있지 않은지 확인하세요.")
        sys.exit(1)


if __name__ == "__main__":
    main()
