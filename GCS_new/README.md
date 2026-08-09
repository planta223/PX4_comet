# Pixhawk4 GCS

최신 `LuaScript/UP_TO_DATE` LoRa 프로토콜에 맞춘 독립 실행용 GCS 사본입니다.

## 사전 설정

`config.py`에서 PC에 연결된 실제 포트를 확인합니다.

```python
SIK_CONNECTION_STRING = "COM11"
LORA_SERIAL_PORT = "COM18"
```

FC와 지상 LoRa 모듈은 모두 `57600/8N1` 투명전송 모드여야 합니다.

## 최초 설치

PowerShell에서 다음을 실행합니다.

```powershell
cd E:\Workspace\Pixhawk4\GCS_new
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

실행 정책 때문에 가상환경 활성화가 차단되면 활성화 없이 다음처럼 설치할 수 있습니다.

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

## 실행

```powershell
cd E:\Workspace\Pixhawk4\GCS_new
.\run_gcs.ps1
```

또는 직접 실행합니다.

```powershell
.\.venv\Scripts\python.exe main.py
```

CSV와 이벤트 로그는 실행 디렉터리의 `logs\` 아래에 생성됩니다.

## 기본 검증

```powershell
cd E:\Workspace\Pixhawk4\GCS_new
.\.venv\Scripts\python.exe -m compileall .
.\.venv\Scripts\python.exe -c "import config, lora, fusion, sik"
```
