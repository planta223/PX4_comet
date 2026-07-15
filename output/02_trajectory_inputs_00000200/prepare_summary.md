# Trajectory CSV Prepare Summary

- Common t0_us: `48597816`

## Output files

- imu: `output\02_trajectory_inputs_00000200\imu.csv`
- gps: `output\02_trajectory_inputs_00000200\gps.csv`
- baro: `output\02_trajectory_inputs_00000200\baro.csv`

## IMU

- input: `output\01_csv_00000200\IMU.csv`
- raw_rows: `28262`
- output_rows: `14131`
- imu_instance: `0`
- available_instances: `[0, 1]`
- time_start_s: `0.0`
- time_end_s: `282.599343`

### Columns

```text
time_s, ax, ay, az, gx, gy, gz
```

## GPS

- input: `output\01_csv_00000200\GPS.csv`
- raw_rows: `1413`
- after_status_rows: `1413`
- after_coord_rows: `1413`
- output_rows: `1413`
- min_status: `3`
- time_start_s: `0.09996`
- time_end_s: `282.519097`

### Columns

```text
time_s, lat, lng, alt, spd, vz, nsats, hdop, status
```

## BARO

- input: `output\01_csv_00000200\BARO.csv`
- raw_rows: `2826`
- output_rows: `2826`
- baro_instance: `0`
- available_instances: `[0]`
- time_start_s: `0.079376`
- time_end_s: `282.579332`

### Columns

```text
time_s, baro_alt, baro_alt_amsl, press, temp, climb_rate, health
```
