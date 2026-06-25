# Trajectory CSV Prepare Summary

- Common t0_us: `137252802`

## Output files

- imu: `output\02_trajectory_inputs_00000197\imu.csv`
- gps: `output\02_trajectory_inputs_00000197\gps.csv`
- baro: `output\02_trajectory_inputs_00000197\baro.csv`

## IMU

- input: `output\01_csv_00000197\IMU.csv`
- raw_rows: `9606`
- output_rows: `4803`
- imu_instance: `0`
- available_instances: `[0, 1]`
- time_start_s: `0.0`
- time_end_s: `96.038904`

### Columns

```text
time_s, ax, ay, az, gx, gy, gz
```

## GPS

- input: `output\01_csv_00000197\GPS.csv`
- raw_rows: `480`
- after_status_rows: `480`
- after_coord_rows: `480`
- output_rows: `480`
- min_status: `3`
- time_start_s: `0.159414`
- time_end_s: `95.959317`

### Columns

```text
time_s, lat, lng, alt, spd, vz, nsats, hdop, status
```

## BARO

- input: `output\01_csv_00000197\BARO.csv`
- raw_rows: `960`
- output_rows: `960`
- baro_instance: `0`
- available_instances: `[0]`
- time_start_s: `0.080341`
- time_end_s: `95.979481`

### Columns

```text
time_s, baro_alt, baro_alt_amsl, press, temp, climb_rate, health
```
