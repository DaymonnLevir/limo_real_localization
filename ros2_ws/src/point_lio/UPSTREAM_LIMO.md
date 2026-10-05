# Vendored source

- URL: https://github.com/LihanChen2004/Point-LIO
- Branch: grid_map_ros2
- Commit: 155f46da0b1eb3fc7c7facf43b00cb124631c807
- Retrieved: 2026-10-05

The upstream C++ algorithm and generic profiles are preserved. LIMO additions:
`config/limo_mid360.yaml`, `launch/limo_mid360.launch.py`,
`rviz_cfg/limo_mid360.rviz`, and this provenance file.

The LIMO profile uses absolute input topics and real time; remaining numerical
parameters initially match upstream MID-360. Extrinsics implement
`p_imu = R_lidar_to_imu * p_lidar + T_lidar_to_imu`
(`src/laserMapping.cpp`, `pointBodyLidarToIMU`).

Actual stationary IMU acceleration magnitude was 0.9951 g, gyro 0.0040 rad/s.
Thus `acc_norm=1.0`. Upstream's conservative rejection limits are retained:
`satu_acc=3.0` in raw g and `satu_gyro=35.0` in rad/s. These are algorithm
rejection thresholds, not a measurement of hardware saturation. Contrary to
the generic YAML comment, Estimator.cpp compares them to the raw components;
acceleration normalization only occurs in the residual calculation.
