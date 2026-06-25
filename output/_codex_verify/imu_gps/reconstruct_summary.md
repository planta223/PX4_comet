# Trajectory Reconstruction Summary

## Mode

- mode: `imu-gps`

## Input files

- IMU CSV: `output\02_trajectory_inputs_00000197\imu.csv`
- GPS CSV: `output\02_trajectory_inputs_00000197\gps.csv`
- BARO CSV: `output\02_trajectory_inputs_00000197\baro.csv`

## Output files

- Trajectory CSV: `output\_codex_verify\imu_gps\trajectory.csv`
- GPS local CSV: `output\_codex_verify\imu_gps\gps_local.csv`
- Summary Markdown: `output\_codex_verify\imu_gps\reconstruct_summary.md`

## Parameters

- dt: `0.02` s (used)
- gravity_z: `9.81` m/s^2 (used)
- gps_blend: `1.0` (used)

## Input ranges

- IMU rows: `4803`
- IMU time: `0.000000` ~ `96.038904` s
- GPS rows: `480`
- GPS time: `0.159414` ~ `95.959317` s
- BARO rows: `960`
- BARO time: `0.080341` ~ `95.979481` s
- BARO interpolation range: `not used`

## Reconstruction result

- Trajectory rows: `4795`
- Trajectory time: `0.159414` ~ `96.038904` s
- Final local position x/y/z: `13.989`, `12.637`, `-189.259` m
- GPS local x range: `-0.443 ~ 16.861` m
- GPS local y range: `-6.234 ~ 17.110` m
- GPS local z range: `-40.140 ~ 0.820` m
- Altitude source: `IMU RK4 integration with GPS correction, experimental`
- GPS correction count: `479`

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

- WARNING: imu-gps mode assumes IMU acceleration is already expressed in an earth-fixed frame. Raw ArduPilot IMU AccX/AccY/AccZ are likely body-frame, so this mode is experimental and not valid for precision trajectory estimation without attitude rotation.
- future work: attitude rotation required
