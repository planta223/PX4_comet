# -*- coding: utf-8 -*-
"""
개발단계 전용 테스트 CLI.

main.py는 실전 비행에만 쓰고, 지상 개발 테스트(사출 명령 SiK/LoRa 분리
검증, 링크 재연결 검증)는 이 스크립트로 완전히 분리했다. main.py의
"사출(전체)" 버튼은 SiK 연결이 있으면 SiK를 우선하고 없으면 LoRa로
폴백하는 실전용 로직이라, 두 경로를 각각 독립적으로 눌러 따로
검증하기가 어렵다 - 여기서는 SiK/LoRa 사출·잠금을 완전히 분리된
메뉴 항목으로 제공한다.

이 실행에서 수신되는 SiK/LoRa 위치 로그(CSV)는 실전 로그(logs/sik_*.csv,
logs/lora_*.csv)와 섞이지 않도록 실행마다 logs/test_run_<타임스탬프>/
전용 폴더에 모아서 기록한다.

사용법: python test_dev.py
"""

import os
import queue
import threading
import time
import types
from datetime import datetime

import config
import fusion
import lora
import mission_features
import shared_state
import sik


def _make_test_config(output_dir):
    """
    config.py 값을 그대로 복사하되 CSV_OUTPUT_DIR만 이번 테스트 실행 전용
    폴더로 바꾼 사본을 만든다. sik.py/lora.py는 cfg를 주입받게 이미
    설계되어 있어(run_sik_pipeline/run_lora_pipeline의 cfg 인자) 코드
    수정 없이 로그 위치만 분리할 수 있다.
    """
    ns = types.SimpleNamespace()
    for name in dir(config):
        if not name.startswith("_"):
            setattr(ns, name, getattr(config, name))
    ns.CSV_OUTPUT_DIR = output_dir
    return ns


class _LinkWatcher:
    """
    백그라운드에서 SiK/LoRa status_queue를 계속 드레인하며 last_seen을
    갱신하고, 생존<->DEAD 전환과 LoRa 커맨드 결과(ACK/실패)를 콘솔에
    출력한다. 라디오를 직접 뽑았다 꽂아보면서 재연결이 실제로 되는지
    눈으로 확인하는 용도.
    """

    def __init__(self, sik_status_q, lora_status_q, cfg):
        self._sik_status_q = sik_status_q
        self._lora_status_q = lora_status_q
        self._cfg = cfg
        self._lock = threading.Lock()
        self.last_sik_seen = None
        self.last_lora_seen = None
        self._sik_alive_prev = None
        self._lora_alive_prev = None

    def sik_alive(self):
        with self._lock:
            last_seen = self.last_sik_seen
        return fusion.is_link_alive(last_seen, self._cfg.SIK_LINK_TIMEOUT_SEC, time.time())

    def lora_alive(self):
        with self._lock:
            last_seen = self.last_lora_seen
        return fusion.is_link_alive(last_seen, self._cfg.LORA_LINK_TIMEOUT_SEC, time.time())

    def run(self):
        while True:
            self._drain_sik()
            self._drain_lora()
            self._report_transitions()
            time.sleep(0.1)

    def _drain_sik(self):
        while True:
            try:
                item = self._sik_status_q.get_nowait()
            except queue.Empty:
                return
            if "last_seen" in item:
                with self._lock:
                    self.last_sik_seen = item["last_seen"]

    def _drain_lora(self):
        while True:
            try:
                item = self._lora_status_q.get_nowait()
            except queue.Empty:
                return
            if "last_seen" in item:
                with self._lock:
                    self.last_lora_seen = item["last_seen"]
            if "command_result" in item:
                result = item["command_result"]
                if result["success"]:
                    print(f"\n[LoRa 커맨드 결과] {result['command_name']} -> {result['result']} ({result['attempts']}회 시도)")
                else:
                    print(f"\n[LoRa 커맨드 결과] {result['command_name']} 실패 - 응답 없음 ({result['attempts']}회 재시도)")

    def _report_transitions(self):
        sik_alive = self.sik_alive()
        if self._sik_alive_prev is not None and sik_alive != self._sik_alive_prev:
            print(f"\n[SiK] {'연결됨' if sik_alive else 'DEAD (재연결 대기 중)'}")
        if self.last_sik_seen is not None:
            self._sik_alive_prev = sik_alive

        lora_alive = self.lora_alive()
        if self._lora_alive_prev is not None and lora_alive != self._lora_alive_prev:
            print(f"\n[LoRa] {'연결됨' if lora_alive else 'DEAD (재연결 대기 중)'}")
        if self.last_lora_seen is not None:
            self._lora_alive_prev = lora_alive


def _fire_sik(pwm_key, label):
    conn = sik.get_connection()
    if conn is None:
        print("SiK 연결 없음 - 명령 전송 안 함")
        return
    for servo in config.SERVO_CHANNELS:
        ch = servo["channel"]
        pwm = servo.get(pwm_key, 1500)
        mission_features.set_servo(conn, ch, pwm)
        print(f"[SiK] CH{ch} ({servo['label']}) -> {pwm}us 전송 (ACK 없음 - 서보 실제 동작 육안/전류로 확인 필요)")
    print(f"[SiK] {label} 명령 전체 전송 완료")


def _fire_lora(command_name, lora_command_q):
    lora.enqueue_command(lora_command_q, command_name)
    print(f"[LoRa] {command_name} 명령 큐에 등록 - 결과는 도착하는 대로 위에 출력됩니다.")


def _print_link_status(watcher, cfg):
    now = time.time()

    def _fmt(last_seen, timeout):
        if last_seen is None:
            return "미수신"
        alive = fusion.is_link_alive(last_seen, timeout, now)
        age = now - last_seen
        return f"{'ALIVE' if alive else 'DEAD'} (마지막 수신 {age:.1f}초 전)"

    print(f"SiK : {_fmt(watcher.last_sik_seen, cfg.SIK_LINK_TIMEOUT_SEC)}")
    print(f"LoRa: {_fmt(watcher.last_lora_seen, cfg.LORA_LINK_TIMEOUT_SEC)}")


_MENU = """
==================== 개발 테스트 메뉴 ====================
 1) SiK 사출 (전체, config.SERVO_CHANNELS pwm_max)
 2) SiK 잠금 (전체, pwm_min)
 3) LoRa 사출 (RELEASE_ALL)
 4) LoRa 잠금 (LOCK_ALL)
 5) 링크 상태 확인 (1회 출력)
 6) 재연결 감시 모드 (라디오 뽑았다/꽂았다 하면서 전환 로그 확인, Enter로 복귀)
 0) 종료
===========================================================
"""


def _drain_trajectories(sik_traj_q, lora_traj_q):
    """
    이 테스트 도구는 궤적 좌표 자체는 안 쓰지만, 큐를 안 비우면 무한정
    쌓이므로 계속 버려준다 (main.py의 _poll_sik/_poll_lora가 하는 소비를
    이 도구에서는 이 역할로 대신함).
    """
    while True:
        try:
            sik_traj_q.get(timeout=0.5)
        except queue.Empty:
            pass
        try:
            lora_traj_q.get_nowait()
        except queue.Empty:
            pass


def main():
    test_dir = os.path.join(config.CSV_OUTPUT_DIR, "test_run_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(test_dir, exist_ok=True)
    test_cfg = _make_test_config(test_dir)
    print(f"이번 테스트 실행의 수신 위치 로그는 여기에 모입니다: {test_dir}")

    shared_origin = shared_state.SharedOrigin()

    sik_traj_q, sik_status_q = queue.Queue(), queue.Queue()
    lora_traj_q, lora_status_q, lora_command_q = queue.Queue(), queue.Queue(), queue.Queue()

    threading.Thread(
        target=sik.run_sik_pipeline,
        args=(sik_traj_q, sik_status_q, test_cfg, shared_origin),
        daemon=True, name="sik-pipeline-test",
    ).start()
    threading.Thread(
        target=lora.run_lora_pipeline,
        args=(lora_traj_q, lora_status_q, test_cfg, lora_command_q, shared_origin),
        daemon=True, name="lora-pipeline-test",
    ).start()
    threading.Thread(target=_drain_trajectories, args=(sik_traj_q, lora_traj_q), daemon=True, name="traj-drain").start()

    watcher = _LinkWatcher(sik_status_q, lora_status_q, test_cfg)
    threading.Thread(target=watcher.run, daemon=True, name="link-watcher").start()

    print(_MENU)
    while True:
        choice = input("선택> ").strip()
        if choice == "1":
            _fire_sik("pwm_max", "사출(전체)")
        elif choice == "2":
            _fire_sik("pwm_min", "잠금(전체)")
        elif choice == "3":
            _fire_lora("RELEASE_ALL", lora_command_q)
        elif choice == "4":
            _fire_lora("LOCK_ALL", lora_command_q)
        elif choice == "5":
            _print_link_status(watcher, test_cfg)
        elif choice == "6":
            print("재연결 감시 모드 - 라디오를 뽑았다 꽂아보며 전환 로그를 확인하세요. Enter를 누르면 메뉴로 돌아갑니다.")
            input()
        elif choice == "0":
            print("종료합니다.")
            break
        else:
            print(_MENU)


if __name__ == "__main__":
    main()
