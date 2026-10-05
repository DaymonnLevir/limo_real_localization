// rotation_parameterization.hpp - 四元数局部参数化(右扰动)
// 职责: 将四元数视为流形(全局空间 4 维 / 切空间 3 维),
//       供 Ceres 以 3 维增量优化四元数参数块.
#pragma once

#include <ceres/ceres.h>

namespace ct_lio::core {

/// 四元数右乘扰动参数化: Plus 定义为 q_new = q * deltaQ(delta),
/// 与 ESKF 中误差状态的旋转更新方式保持一致
class RotationParameterization : public ceres::LocalParameterization {
 public:
  bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;
  bool ComputeJacobian(const double* x, double* jacobian) const override;
  int GlobalSize() const override { return 4; }
  int LocalSize() const override { return 3; }
};

}  // namespace ct_lio::core
