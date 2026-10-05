// frame_types.hpp - 核心数据结构(帧级)
// 定义连续时间优化所需的数据:
//   Point3D          单点(雷达系原始点 + 世界系点 + 时间信息);
//   State            一帧的 begin/end 双位姿状态及速度/零偏;
//   CloudFrame       一帧点云(世界系匹配点 + 雷达系原始点);
//   MeasurementGroup 一次激光测量对应的全部 IMU 数据.
#pragma once

#include <deque>
#include <vector>

#include "ct_lio_ros2/core/imu_measurement.hpp"
#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

/// 单点结构(带时间信息)
struct Point3D {
  Eigen::Vector3d raw_point = Eigen::Vector3d::Zero();  // 雷达系原始坐标
  Eigen::Vector3d point = Eigen::Vector3d::Zero();      // 世界系坐标(由变换填充)
  double intensity = 0.0;
  double alpha_time = 0.0;    // 相对帧首归一化时刻 [0,1]
  double relative_time = 0.0; // 相对帧首时间(s)
};

/// 状态: 帧尾(end)优化变量 + 帧首(begin)状态(连续时间轨迹的起点)
struct State {
  State() = default;

  // 帧尾(end)状态: 当前帧的优化变量
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  // 帧首(begin)状态: 连续时间轨迹的起点
  Eigen::Quaterniond rotation_begin = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation_begin = Eigen::Vector3d::Zero();

  /// 深拷贝(用于历史帧存档)
  State Clone() const {
    State s;
    s.rotation = rotation;
    s.translation = translation;
    s.rotation_begin = rotation_begin;
    s.translation_begin = translation_begin;
    return s;
  }

  /// 滚动到下一帧: 当前 end 状态成为下一帧的 begin
  void RollToNextFrame() {
    rotation_begin = rotation;
    translation_begin = translation;
  }
};

/// 一帧点云: 世界系点(优化/匹配/建图用)
struct CloudFrame {
  int frame_id = 0;

  State state;  // 以值语义保存状态

  std::vector<Point3D> point_surf;
};

/// 测量组: 一帧激光点云 + 时间同步的 IMU 数据
struct MeasurementGroup {
  double lidar_end_time = 0;
  std::vector<Point3D> lidar;
  std::deque<ImuPtr> imu;
};

}  // namespace ct_lio::core
