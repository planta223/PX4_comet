# Trajectory Reconstruction Summary

## Mode

- mode: `gps-baro`

## Input files

- IMU CSV: `output\02_trajectory_inputs_00000197\imu.csv`
- GPS CSV: `output\02_trajectory_inputs_00000197\gps.csv`
- BARO CSV: `output\02_trajectory_inputs_00000197\baro.csv`

## Output files

- Trajectory CSV: `output\03_trajectory_00000197\trajectory.csv`
- GPS local CSV: `output\03_trajectory_00000197\gps_local.csv`
- Summary Markdown: `output\03_trajectory_00000197\reconstruct_summary.md`

## Parameters

- dt: `0.02` s (not used)
- gravity_z: `9.81` m/s^2 (not used)
- gps_blend: `1.0` (not used)

## Input ranges

- IMU rows: `4803`
- IMU time: `0.000000` ~ `96.038904` s
- GPS rows: `480`
- GPS time: `0.159414` ~ `95.959317` s
- BARO rows: `960`
- BARO time: `0.080341` ~ `95.979481` s
- BARO interpolation range: `0.080341 ~ 95.979481 s`

## Reconstruction result

- Trajectory rows: `480`
- Trajectory time: `0.159414` ~ `95.959317` s
- Final local position x/y/z: `13.567`, `13.882`, `-0.215` m
- GPS local x range: `-0.443 ~ 16.861` m
- GPS local y range: `-6.234 ~ 17.110` m
- GPS local z range: `-40.140 ~ 0.820` m
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
