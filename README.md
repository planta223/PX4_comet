# Pixhawk4 Rocket Avionics

> **Date:** 2026-08-08<br>
> **Competition:** 2026 전국대학생 로켓발사대회

[![Competition Video](https://img.youtube.com/vi/SQCmV_QEpsk/hqdefault.jpg)](https://www.youtube.com/watch?v=SQCmV_QEpsk)

Pixhawk 4와 ArduPilot Rover를 기반으로 로켓 에비오닉스의 낙하산 사출, 센서 로그 저장, 실시간 텔레메트리 수신, 비행 후 로그 후처리 및 궤적 재구성을 수행한 프로젝트입니다.

## 1. 프로젝트 목적

* Pixhawk 4를 로켓용 센서 로깅 및 텔레메트리 플랫폼으로 사용
* ArduPilot DataFlash 로그(`.BIN`)에서 센서별 데이터 추출
* SiK MAVLink 기반 Mission Planner 지상국 운용
* LoRa/RoLa 기반 백업 회수용 텔레메트리 구성
* 비행 후 SD 로그 기반 궤적 재구성

## 2. 현재 확인된 시스템 상태

| 항목                | 상태                                               |
| ----------------- | ------------------------------------------------ |
| Flight Controller | Pixhawk 4                                        |
| Firmware          | ArduRover V4.6.0                                 |
| Ground Station    | Mission Planner 1.3.83                           |
| USB 연결            | COM10                                            |
| SiK 연결            | TELEM1 → COM9 / 57600                            |
| GPS 상태            | 3D Fix 확인, `gpsstatus=4`, HDOP 양호                |
| SD 로그             | `LOG_DISARMED=Enabled` 설정 후 `00000197.BIN` 생성 확인 |
| Lua Scripting     | `SCR_ENABLE=1` 확인                                |

## 3. 저장소 구조

```text
Pixhawk4/
├── 000_SD_backup_20260625/
│   └── ...                       # SD 카드 전체 백업, Git 제외
├── LOGS/
│   └── 00000197.BIN              # 테스트 로그 후보
├── output/
│   └── ...                       # BIN → CSV 변환 결과
├── scripts/
│   └── ...                       # 로그 변환 및 후처리 스크립트
├── trajectory_input/
│   └── ...                       # 궤적 재구성 입력 CSV
├── folder_discription.txt
├── .gitignore
└── README.md
```

## 4. 데이터 처리 흐름

```text
Pixhawk SD DataFlash Log (.BIN)
        ↓
BIN → CSV 변환
        ↓
GPS.csv / IMU.csv / BARO.csv / ATT.csv
        ↓
궤적 입력 CSV 정리
        ↓
trajectory_input/
        ↓
Python 궤적 재구성
        ↓
reconstructed_trajectory.csv / trajectory plot
```

현재 단계에서는 궤적 재구성보다 먼저 `.BIN` 파일에서 센서별 CSV를 안정적으로 추출하는 것을 목표로 합니다.

## 5. 통신 및 저장 경로 역할

| 경로          | 역할                | 데이터                                        |
| ----------- | ----------------- | ------------------------------------------ |
| Local SD    | 원본 로그 저장 및 후처리 기준 | GPS, IMU, BARO, ATT, EKF, RCOU, POWR, VIBE |
| SiK MAVLink | 실시간 지상국 모니터링      | 위치, 고도, 자세, GPS 상태, 이벤트                    |
| LoRa/RoLa   | 백업 회수용 저대역 링크     | 상태, 마지막 GPS 좌표, 고도, 이벤트                    |

## 6. SD 로그 관리 원칙

* 원본 `.BIN` 로그는 수정하지 않습니다.
* SD 카드 전체 백업 폴더는 Git에 포함하지 않습니다.
* 변환 결과는 `output/` 또는 `trajectory_input/`에 저장합니다.
* 필요한 경우 테스트용 `.BIN`만 별도로 관리합니다.
* 대용량 원본 로그는 팀 합의 없이 Git에 추가하지 않습니다.

## 7. 다음 작업

### 7.1 BIN → CSV 변환 스크립트 작성

예정 파일:

```text
scripts/export_bin_to_csv.py
```

목표:

* `LOGS/00000197.BIN`에서 메시지별 CSV 추출
* 우선 추출 대상:

  * `GPS`
  * `IMU`
  * `BARO`
  * `ATT`

예상 실행 예:

```bash
python scripts/export_bin_to_csv.py LOGS/00000197.BIN --types GPS IMU BARO ATT
```

### 7.2 궤적 입력 CSV 정리

예정 파일:

```text
scripts/prepare_trajectory_inputs.py
```

목표:

* `GPS.csv` → `trajectory_input/00000197/gps_input.csv`
* `IMU.csv` → `trajectory_input/00000197/imu_input.csv`

예상 입력 형식:

```text
gps_input.csv:
time_s, lat, lng, alt

imu_input.csv:
time_s, ax, ay, az
```

### 7.3 궤적 재구성

CSV 입력을 기반으로 GPS/IMU/BARO/ATT 데이터를 검증한 후, 궤적 재구성 프로그램으로 2D/3D 궤적과 고도 그래프를 생성합니다.

## 8. 주의사항

* Pixhawk 4는 본 프로젝트에서 드론 제어기가 아니라 센서 로깅 및 텔레메트리 플랫폼으로 사용합니다.
* 정밀 궤적 재구성은 실시간이 아니라 SD 로그 후처리 기반으로 수행합니다.
* SiK는 주 텔레메트리 링크, LoRa/RoLa는 백업 회수 링크로 분리합니다.
* LoRa/RoLa 연결은 물리 배선, 전원, UART 레벨, 패킷 포맷 통일이 필요합니다.
* 사출 서보 테스트는 USB 전원만으로 수행하지 않습니다.
