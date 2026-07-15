# ArduPilot EKF3 BIN Replay

ArduPilot DataFlash `.BIN` 파일을 `pymavlink`로 직접 읽고, EKF3의 위치·속도·자세·health 상태를 XKF1 시간축에 정렬해 Matplotlib 3D 애니메이션으로 재생하는 독립 Python 프로젝트입니다. CSV나 외부 궤적 재구성 결과는 입력으로 사용하지 않습니다.

이 화면은 실제 참값(ground truth)이 아니라 **ArduPilot EKF가 기록한 추정값의 재생**입니다.

## 설치

Python 3.9 이상을 권장합니다. 프로젝트 디렉터리에서 가상환경을 만들고 의존성을 설치합니다.

```bash
cd bin_ekf_replay
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## 실행

```bash
python replay_bin_ekf.py ../LOGS/00000200.BIN
python replay_bin_ekf.py ../LOGS/00000200.BIN --core 0 --fps 30 --speed 1.0
python replay_bin_ekf.py ../LOGS/00000200.BIN --summary-only
python replay_bin_ekf.py ../LOGS/00000200.BIN --summary-only --save-csv output/ekf_state.csv
```

전체 옵션:

```text
--core 0
--fps 30
--speed 1.0
--trail-seconds 30
--match-tolerance 0.05
--save-csv output/ekf_state.csv
--no-health
--use-quaternion-conjugate
--summary-only
```

`--match-tolerance`는 XKQ/XKF4 샘플을 가장 가까운 XKF1 timestamp에 붙일 때 허용하는 초 단위 차이입니다. quaternion은 보간하지 않고 nearest-neighbor로만 매칭합니다. `--summary-only`는 GUI 없이 메시지 수, 범위, quaternion 유효 비율과 status 통계를 출력합니다.

## DataFlash 메시지의 역할

- `XKF1`: 기준 시간축과 EKF core 번호, Euler 자세, NED 속도, NED 위치를 제공합니다. 구현은 `TimeUS`, `C`, `Roll/Pitch/Yaw`, `VN/VE/VD`, `PN/PE/PD`와 가능한 경우 `GX/GY/GZ/OH`를 읽습니다.
- `XKQ`: EKF quaternion `Q1/Q2/Q3/Q4`를 제공합니다. 이 프로젝트에서는 각각 `qw/qx/qy/qz`(scalar-first)로 취급하고 정규화합니다. 유효하지 않은 샘플은 직전 정상 quaternion을 유지하며, XKQ 자체가 없으면 XKF1 Euler angle로 자세를 만듭니다.
- `XKF4`: innovation test ratio `SV/SP/SH/SM`과 `FS/TS/SS` fault/timeout/solution status를 제공합니다. 로그 정의에 없는 필드는 `NaN`으로 두며 실제 발견 필드명을 요약에 출력합니다. XKF4가 없어도 궤적 재생은 계속됩니다.
- `ORGN`: EKF 원점(위도·경도·고도 등)을 진단용으로 읽습니다. 현재 로컬 NED 궤적 재생에는 원점 변환을 적용하지 않습니다.
- `MSG`: 펌웨어 및 부팅 텍스트를 진단용으로 읽습니다.

모든 상대시간 `time_s`는 첫 유효 XKF1 `TimeUS`를 0초로 삼습니다. 비단조 timestamp는 정렬하고 중복 timestamp는 마지막 샘플을 사용합니다.

## NED에서 화면 ENU로 변환

ArduPilot EKF의 로컬 위치·속도는 North-East-Down(NED)입니다. 화면은 East-North-Up 형태로 다음처럼 표시합니다.

```text
x_east  = PE       vx_east  = VE
y_north = PN       vy_north = VN
z_up    = -PD      vz_up    = -VD
```

CSV에는 원본 NED 컬럼과 이 ENU 컬럼을 모두 저장합니다. 자세의 body 축도 같은 NED→ENU 고정 행렬을 적용합니다.

## Quaternion 방향 검증

기본 해석은 XKQ를 body→NED 회전으로 보고, 그 뒤 NED→ENU 화면 변환을 적용합니다. 펌웨어 버전이나 해석 convention에 따라 반대로 보이면 `--use-quaternion-conjugate`를 켜 비교하십시오. 화면의 빨간 `body X`(기체 전방), 초록 `body Y`(우측), 파란 `body Z`(아래) 라벨을 비행 방향, 상승/하강 구간, Roll/Pitch/Yaw 값과 대조하는 것이 가장 확실합니다.

축만 그리므로 모델 offset 기본값은 identity입니다. 다른 모델 축을 붙일 경우 `state_model.py`의 `MODEL_AXIS_OFFSET` 고정 회전행렬을 바꿀 수 있습니다. STL 모델은 사용하지 않습니다.

## 키보드 조작

- `Space`: 재생/일시정지
- `R`: 처음부터 재시작
- `Right Arrow` / `Left Arrow`: 1초 앞/뒤로 이동
- `Up Arrow` / `Down Arrow`: 재생 배속 2배 증가/감소
- `Esc`: 종료

현재 재생 상태와 배속은 화면 아래에 표시됩니다. `fps`는 화면 갱신 빈도이고 `speed`는 로그 시간의 진행 배율이므로 원본 EKF 샘플 주기와 독립적입니다.

## 알려진 한계

- DataFlash 필드 구성과 quaternion convention은 ArduPilot 버전 및 vehicle/firmware 설정에 따라 달라질 수 있습니다. 프로그램은 예상 필드가 없을 때 실제 필드명을 출력하지만, 의미가 바뀐 새 필드를 자동 추론하지는 않습니다.
- nearest 매칭 허용 오차 밖의 XKQ/XKF4는 결측 처리됩니다. quaternion 결측은 이전 자세를 유지합니다.
- Euler fallback은 quaternion과 동일한 기록 경로가 아니며 Euler convention/랩 구간의 영향을 받습니다.
- ORGN은 읽기만 하며 WGS84 절대좌표로 변환하지 않습니다.
- 손상 레코드는 가능한 범위에서 건너뛰지만, pymavlink가 스트림을 복구하지 못하면 남은 로그를 모두 읽지 못할 수 있습니다.
- Matplotlib 3D는 진단용이며 실시간 비행 제어 또는 고정밀 렌더러가 아닙니다.
