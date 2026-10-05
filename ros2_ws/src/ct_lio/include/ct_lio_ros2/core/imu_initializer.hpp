// imu_initializer.hpp - IMU 初始化器
//
// 职责: 在启动阶段完成 IMU 零偏与重力的初始化, 供 ESKF 设置初始条件.
// 支持 static / dynamic 双模式, 采用哪种由外部配置决定:
//   static:   要求载体保持静止, 通过噪声阈值做静止判定, 估计零偏与重力,
//             精度较高, 但必须在静止状态下启动;
//   dynamic:  无需静止, 攒够一定数量的 IMU 数据即可, 只估计重力方向,
//             bg/ba 置零后由滤波器在线学习.
#pragma once

#include <deque>

#include "ct_lio_ros2/core/imu_measurement.hpp"
#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

/// IMU 初始化器
class ImuInitializer {
 public:
  struct Options {
    double init_time_seconds = 1.0;    // static: 静止收集时间
    int init_imu_queue_max_size = 2000;
    double max_static_gyro_var = 0.5;  // static: 静止判定阈值
    double max_static_acce_var = 0.6;
    double gravity_norm = 9.81;        // 重力大小(可配置)
    int dynamic_min_imu_count = 200;   // dynamic: 攒够 IMU 数
  };

  explicit ImuInitializer(const Options& opt, bool use_static = true)
      : options_(opt), init_mode_static_(use_static) {}

  /// 逐帧喂入 IMU 数据; 返回 true 表示初始化完成(按构造时选定的模式运行)
  bool AddImu(const ImuMeasurement& imu);

  bool InitSuccess() const { return init_success_; }

  Vec3d GetInitBg() const { return init_bg_; }
  Vec3d GetInitBa() const { return init_ba_; }
  Vec3d GetGravity() const { return gravity_; }

 private:
  // static 模式: 静止检测 + 估计零偏/重力
  bool TryInitStatic();
  // dynamic 模式: 攒够数据即完成, 重力方向取加速度均值, bg/ba 置零
  bool TryInitDynamic();

  Options options_;
  bool init_success_ = false;
  std::deque<ImuMeasurement> imu_deque_;
  double init_start_time_ = 0.0;

  Vec3d cov_gyro_ = Vec3d::Zero();
  Vec3d cov_acce_ = Vec3d::Zero();
  Vec3d init_bg_ = Vec3d::Zero();
  Vec3d init_ba_ = Vec3d::Zero();
  Vec3d gravity_ = Vec3d(0, 0, -9.81);
  bool init_mode_static_ = true;   // 模式开关, 由外部配置决定
};

}  // namespace ct_lio::core
