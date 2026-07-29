# COMET LoRa 텔레메트리 주기·손실률 시험

## 안전 조건

- 지상 시험 전에 `CONFIG.TRANSITION.ASCENT_TO_DEPLOY.DEPLOY_ENABLE = false`인지 확인한다.
- 서보와 사출 장치의 전원·기계적 연결 상태를 별도로 확인한다.
- 이 시험 설정은 텔레메트리 주기만 바꾸며 상태 전이, 명령 ID, ACK, CRC 및 패킷 필드는 변경하지 않는다.

## 시험 주기 설정

`LuaScript/modules/comet_config.lua`에서 한 번의 시험에 사용할 주기 하나만 지정한다.

```lua
CONFIG.LORA.TEST = {
    ENABLED = true,
    PERIOD_MS = 500
}
```

시험 대상 주기는 `250`, `500`, `750`, `1000`, `1500`, `2000` ms이다. 자동 순환 기능은 없으므로 주기를 변경할 때마다 Lua 파일을 다시 배포하고 Pixhawk를 재부팅한다.

`TEST.ENABLED = false`이면 기존 동작대로 ASCENT/DEPLOY/DESCENT에서는 `FAST_PERIOD_MS`, READY/LANDED에서는 `NORMAL_PERIOD_MS`를 사용한다.

## 시험 절차

1. `CONFIG.LORA.TEST.ENABLED = true`로 설정한다.
2. `CONFIG.LORA.TEST.PERIOD_MS`에 시험 주기를 지정한다.
3. 현재 Lua 파일을 Pixhawk에 배포한다.
4. Pixhawk를 재부팅한다.
5. PC에서 같은 예상 주기로 수신기를 실행한다.

   ```powershell
   python lora_receiver.py --port COM5 --baud 57600 --expected-period-ms 500 --packet-length 34
   ```

6. 최소 2~5분 동안 수신한다.
7. `Ctrl+C`로 종료한다.
8. 텔레메트리 CSV와 `lora_test_summary_<timestamp>.json`을 확인한다.
9. 나머지 시험 주기로 변경하여 같은 절차를 반복한다.

누적 통계 출력 주기와 summary 경로는 필요하면 지정할 수 있다.

```powershell
python lora_receiver.py `
    --port COM5 `
    --baud 57600 `
    --expected-period-ms 500 `
    --packet-length 34 `
    --stats-interval-s 5 `
    --csv lora_test_500ms.csv `
    --summary-json lora_test_summary_500ms.json
```

CSV를 저장하지 않고 콘솔과 summary만 확인하려면 `--no-csv`를 추가한다.

## 결과 해석

- `received_count`: CRC 검증을 통과한 전체 수신 패킷 수
- `missed_count`: 정상 순방향 sequence 간격에서 계산된 누락 수
- `duplicate_count`: 이전 정상 기준과 같은 sequence 수
- `out_of_order_count`: 역전되거나 오래된 sequence 수
- `loss_rate_percent`: `missed_count / total_expected_count × 100`
- `interval_ms`: 연속 CRC 정상 패킷의 PC monotonic 수신 간격
- `jitter_ms`: `abs(interval_ms - expected_period_ms)`
- `average_payload_rate_Bps`: `received_count × packet_length / duration_s`
- `estimated_required_payload_rate_Bps`: `packet_length × 1000 / expected_period_ms`

순서 역전 패킷은 누락 수로 계산하지 않으며, 다음 정상 패킷의 비교 기준도 변경하지 않는다.

## 주기별 권장 기록

| 주기(ms) | 시험시간(s) | 예상 패킷 수 | 수신 패킷 수 | 누락 수 | 손실률(%) | 평균 간격(ms) | 최대 간격(ms) | 표준편차(ms) | 평균 B/s |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 250 |  |  |  |  |  |  |  |  |  |
| 500 |  |  |  |  |  |  |  |  |  |
| 750 |  |  |  |  |  |  |  |  |  |
| 1000 |  |  |  |  |  |  |  |  |  |
| 1500 |  |  |  |  |  |  |  |  |  |
| 2000 |  |  |  |  |  |  |  |  |  |

## 자체 검증

COM 포트를 열지 않고 패킷 디코딩, CRC 복구, sequence 누락·중복·wrap-around·순서 역전, CSV 및 summary 저장을 검증한다.

```powershell
python lora_receiver.py --self-test
```
