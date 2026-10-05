// nav_state.hpp - 导航状态(核心数据)
#pragma once

#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

/// 导航状态: 时间 + R(SO3) + p + v + bg + ba
template <typename T>
struct NavState {
  using Vec3 = Eigen::Matrix<T, 3, 1>;
  using SO3 = Sophus::SO3<T>;

  NavState() = default;

  explicit NavState(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(),
                    const Vec3& v = Vec3::Zero(), const Vec3& bg = Vec3::Zero(),
                    const Vec3& ba = Vec3::Zero())
      : timestamp(time), R(R), p(t), v(v), bg(bg), ba(ba) {}

  NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
      : timestamp(time), R(pose.so3()), p(pose.translation()), v(vel) {}

  double timestamp = 0;
  SO3 R;
  Vec3 p = Vec3::Zero();
  Vec3 v = Vec3::Zero();
  Vec3 bg = Vec3::Zero();
  Vec3 ba = Vec3::Zero();
};

using NavStated = NavState<double>;

}  // namespace ct_lio::core
