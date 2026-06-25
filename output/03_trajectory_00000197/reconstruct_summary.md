# Trajectory Reconstruction Summary

## Inputs

- IMU CSV: `output\02_trajectory_inputs_00000197\imu.csv`
- GPS CSV: `output\02_trajectory_inputs_00000197\gps.csv`
- BARO CSV: `output\02_trajectory_inputs_00000197\baro.csv`

## Outputs

- Trajectory CSV: `output\03_trajectory_00000197\trajectory.csv`
- GPS local CSV: `output\03_trajectory_00000197\gps_local.csv`

## Parameters

- dt: `0.02` s
- gravity_z: `9.81` m/s^2
- gps_blend: `1.0`

## Input ranges

- IMU rows: `4803`
- IMU time: `0.000000` ~ `96.038904` s
- GPS rows: `480`
- GPS time: `0.159414` ~ `95.959317` s
- BARO rows: `960`
- BARO time: `0.080341` ~ `95.979481` s

## Reconstruction result

- Trajectory rows: `4795`
- Trajectory time: `0.159414` ~ `96.038904` s
- GPS correction count: `479`
- Start lat/lng: `37.29684700`, `126.83464900`
- End lat/lng: `37.29696052`, `126.83480697`
- Final local position x/y/z: `13.989`, `12.637`, `-189.259` m

## Columns

### trajectory

```text
time_s, x, y, z, vx, vy, vz, lat, lng
```

### gps_local

```text
time_s, lat, lng, alt, spd, vz, nsats, hdop, status, gx, gy, gz
```

## Important limitation

- This reconstruction assumes IMU acceleration is already expressed in an earth-fixed frame. If AccX/AccY/AccZ are body-frame raw IMU values, attitude-based rotation using ATT/XKQ is required before precision use.
