// imu_measurement.hpp - IMU 测量结构(核心数据)
#pragma once

#include <memory>

#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

/// IMU 单次测量
struct ImuMeasurement {
  ImuMeasurement() = default;
  ImuMeasurement(double t, const Vec3d& gyro, const Vec3d& acce)
      : timestamp(t), gyro(gyro), acce(acce) {}

  double timestamp = 0.0;   // 秒
  Vec3d gyro = Vec3d::Zero();  // 角速度(rad/s)
  Vec3d acce = Vec3d::Zero();  // 加速度(m/s^2)
  bool saturated = false;      // 本帧是否饱和(由检测层标记)
};

using ImuPtr = std::shared_ptr<ImuMeasurement>;

}  // namespace ct_lio::core
