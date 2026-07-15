# Trajectory Reconstruction Summary

## Mode

- mode: `gps-baro`

## Input files

- IMU CSV: `output\02_trajectory_inputs_00000200\imu.csv`
- GPS CSV: `output\02_trajectory_inputs_00000200\gps.csv`
- BARO CSV: `output\02_trajectory_inputs_00000200\baro.csv`

## Output files

- Trajectory CSV: `output\03_trajectory_00000200\trajectory.csv`
- GPS local CSV: `output\03_trajectory_00000200\gps_local.csv`
- Summary Markdown: `output\03_trajectory_00000200\reconstruct_summary.md`

## Parameters

- dt: `0.02` s (not used)
- gravity_z: `9.81` m/s^2 (not used)
- gps_blend: `1.0` (not used)

## Input ranges

- IMU rows: `14131`
- IMU time: `0.000000` ~ `282.599343` s
- GPS rows: `1413`
- GPS time: `0.099960` ~ `282.519097` s
- BARO rows: `2826`
- BARO time: `0.079376` ~ `282.579332` s
- BARO interpolation range: `0.079376 ~ 282.579332 s`

## Reconstruction result

- Trajectory rows: `1413`
- Trajectory time: `0.099960` ~ `282.519097` s
- Final local position x/y/z: `4.959`, `2.750`, `-0.047` m
- GPS local x range: `-93.656 ~ 5.065` m
- GPS local y range: `-25.359 ~ 92.462` m
- GPS local z range: `-2.470 ~ 1.990` m
- Altitude source: `BARO relative altitude interpolated to GPS timestamps`
- GPS correction count: `not used`

## Columns

### trajectory

```text
time_s, x, y, z, vx, vy, vz, lat, lng, source_mode, alt_source
```

### gps_local

```text
time_s, lat, lng, alt, spd, vz, nsats, hdop, status, gx, gy, gz
```

## Limitation / Warning

- This result is intended as a conservative GPS/BARO-based sanity-check trajectory rather than a precision inertial solution.
