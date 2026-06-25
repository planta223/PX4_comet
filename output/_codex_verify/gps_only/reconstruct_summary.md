# Trajectory Reconstruction Summary

## Mode

- mode: `gps-only`

## Input files

- IMU CSV: `output\02_trajectory_inputs_00000197\imu.csv`
- GPS CSV: `output\02_trajectory_inputs_00000197\gps.csv`
- BARO CSV: `N/A`

## Output files

- Trajectory CSV: `output\_codex_verify\gps_only\trajectory.csv`
- GPS local CSV: `output\_codex_verify\gps_only\gps_local.csv`
- Summary Markdown: `output\_codex_verify\gps_only\reconstruct_summary.md`

## Parameters

- dt: `0.02` s (not used)
- gravity_z: `9.81` m/s^2 (not used)
- gps_blend: `1.0` (not used)

## Input ranges

- IMU rows: `4803`
- IMU time: `0.000000` ~ `96.038904` s
- GPS rows: `480`
- GPS time: `0.159414` ~ `95.959317` s
- BARO rows: `N/A`
- BARO time: `N/A`
- BARO interpolation range: `not used`

## Reconstruction result

- Trajectory rows: `480`
- Trajectory time: `0.159414` ~ `95.959317` s
- Final local position x/y/z: `13.567`, `13.882`, `-40.140` m
- GPS local x range: `-0.443 ~ 16.861` m
- GPS local y range: `-6.234 ~ 17.110` m
- GPS local z range: `-40.140 ~ 0.820` m
- Altitude source: `GPS relative altitude`
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
