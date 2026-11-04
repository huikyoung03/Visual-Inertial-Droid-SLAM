# Device profiles

Each profile describes one physical camera/IMU rig. A different S20 rear lens,
crop, orientation, output resolution, stabilization setting, or camera app
requires a different profile and calibration.

`galaxy_s20_rear_wide_640x480.json` is deliberately non-runnable. Fill the
following fields using measurements from the exact capture configuration, then
change `status` to `calibrated`:

- `camera.intrinsics`: `[fx, fy, cx, cy]` in the saved 640x480 pixel grid.
- `camera.distortion`: DROID order `k1, k2, p1, p2[, k3[, k4, k5, k6]]`.
- `imu.R_imu_to_camera`: row-major 3x3 matrix; `v_camera = R_CI * v_imu`.
- `imu.t_imu_to_camera_m`: IMU origin in camera coordinates, in metres.
- `imu.camera_imu_time_offset_sec`, stationary bias and noise/random-walk
  measurements in the units recorded in the profile. The convention is
  `t_imu_aligned = t_imu_raw + camera_imu_time_offset_sec`.
- `imu.camera_imu_time_offset_convention` must be exactly
  `imu_aligned_time = imu_raw_time + offset`; this prevents an estimator using
  the opposite reporting convention from being copied with the wrong sign.
- `imu.accel_scale` is applied as `a_corrected = scale * (a_raw - bias)` during
  IMU preintegration when supplied.

The web collector uses browser timestamps, so it is suitable for calibration
and smoke tests only. A production profile should be used with a native Android
collector that records Camera2 and SensorEvent monotonic timestamps. Send a
single common `timestamp_ns` time base for both camera and IMU, declared as
`timestamp_source: "android_sensor_nanoseconds"`.
