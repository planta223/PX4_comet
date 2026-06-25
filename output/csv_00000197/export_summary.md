# BIN to CSV Export Summary

- Input BIN: `LOGS\00000197.BIN`
- Output directory: `output\csv_00000197`

## Exported message types

| Type | Output CSV | Rows | Size(bytes) | Status |
| --- | --- | ---: | ---: | --- |
| GPS | `output\csv_00000197\GPS.csv` | 480 | 64029 | OK |
| IMU | `output\csv_00000197\IMU.csv` | 9606 | 1666592 | OK |
| BARO | `output\csv_00000197\BARO.csv` | 960 | 122782 | OK |

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
