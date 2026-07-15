# BIN to CSV Export Summary

- Input BIN: `LOGS\00000200.BIN`
- Output directory: `output\01_csv_00000200`

## Exported message types

| Type | Output CSV | Rows | Size(bytes) | Status |
| --- | --- | ---: | ---: | --- |
| GPS | `output\01_csv_00000200\GPS.csv` | 1413 | 188261 | OK |
| IMU | `output\01_csv_00000200\IMU.csv` | 28262 | 4717177 | OK |
| BARO | `output\01_csv_00000200\BARO.csv` | 2826 | 363379 | OK |

## CSV headers

### GPS

```text
Type, TimeUS, I, Status, GMS, GWk, NSats, HDop, Lat, Lng, Alt, Spd, GCrs, VZ, Yaw, U
```

### IMU

```text
Type, TimeUS, I, GyrX, GyrY, GyrZ, AccX, AccY, AccZ, EG, EA, T, GH, AH, GHz, AHz
```

### BARO

```text
Type, TimeUS, I, Alt, AltAMSL, Press, Temp, CRt, SMS, Offset, GndTemp, Health
```
