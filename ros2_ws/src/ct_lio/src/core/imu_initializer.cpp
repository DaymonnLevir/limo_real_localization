// imu_initializer.cpp - IMU 初始化器实现
// 两种模式的完整流程说明见 imu_initializer.hpp 头注释.
#include "ct_lio_ros2/core/imu_initializer.hpp"

#include <cmath>

#include "ct_lio_ros2/utils/math_utils.hpp"

namespace ct_lio::core {

bool ImuInitializer::AddImu(const ImuMeasurement& imu) {
  if (init_success_) return true;

  if (imu_deque_.empty()) {
    init_start_time_ = imu.timestamp;
  }
  imu_deque_.push_back(imu);
  while (imu_deque_.size() > static_cast<size_t>(options_.init_imu_queue_max_size)) {
    imu_deque_.pop_front();
  }

  // 按选定模式尝试初始化
  if (init_mode_static_) {
    if (TryInitStatic()) init_success_ = true;
  } else {
    if (TryInitDynamic()) init_success_ = true;
  }
  return init_success_;
}

bool ImuInitializer::TryInitStatic() {
  if (imu_deque_.size() < 10) return false;
  double init_time = imu_deque_.back().timestamp - init_start_time_;
  if (init_time < options_.init_time_seconds) return false;

  Vec3d mean_gyro, mean_acce;
  computeMeanAndCovDiag(imu_deque_, mean_gyro, cov_gyro_,
                              [](const ImuMeasurement& imu) { return imu.gyro; });
  computeMeanAndCovDiag(imu_deque_, mean_acce, cov_acce_,
                              [](const ImuMeasurement& imu) { return imu.acce; });

  // 重力: 取加速度均值方向(静止时加速度即反重力), 大小用参数指定
  gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm;

  // 扣除重力后重新计算加计方差(这才是真实噪声)
  computeMeanAndCovDiag(imu_deque_, mean_acce, cov_acce_,
                              [this](const ImuMeasurement& imu) { return imu.acce + gravity_; });

  // 静止性验证: 噪声超过阈值说明载体仍在运动, 放弃本轮, 待下帧重试
  if (cov_gyro_.norm() > options_.max_static_gyro_var) return false;
  if (cov_acce_.norm() > options_.max_static_acce_var) return false;

  init_bg_ = mean_gyro;
  init_ba_ = mean_acce;
  imu_deque_.clear();
  return true;
}

bool ImuInitializer::TryInitDynamic() {
  // dynamic 模式: 攒够 IMU 数据即认为可用, 重力方向取加速度均值, bg/ba 置零后在线学习
  if (imu_deque_.size() < static_cast<size_t>(options_.dynamic_min_imu_count)) return false;

  Vec3d mean_acce = Vec3d::Zero();
  for (const auto& imu : imu_deque_) mean_acce += imu.acce;
  mean_acce /= static_cast<double>(imu_deque_.size());

  gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm;
  init_bg_.setZero();
  init_ba_.setZero();
  imu_deque_.clear();
  return true;
}

}  // namespace ct_lio::core
