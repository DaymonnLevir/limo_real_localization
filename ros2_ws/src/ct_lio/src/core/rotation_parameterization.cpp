// rotation_parameterization.cpp - 四元数局部参数化实现(Ceres)
#include "ct_lio_ros2/core/rotation_parameterization.hpp"

#include <Eigen/Geometry>

namespace ct_lio::core {

bool RotationParameterization::Plus(const double* x, const double* delta,
                                    double* x_plus_delta) const {
  Eigen::Map<const Eigen::Quaterniond> q(x);
  Eigen::Map<const Eigen::Vector3d> dp(delta);

  // 将三维扰动向量映射为小角度四元数(旋转向量取半并归一化)
  Eigen::Quaterniond dq;
  dq.w() = 1.0;
  dq.x() = dp.x() * 0.5;
  dq.y() = dp.y() * 0.5;
  dq.z() = dp.z() * 0.5;
  dq.normalize();

  // 右乘扰动并归一化, 保证输出仍是单位四元数
  Eigen::Map<Eigen::Quaterniond> q_out(x_plus_delta);
  q_out = (q * dq).normalized();
  return true;
}

bool RotationParameterization::ComputeJacobian(const double* /*x*/, double* jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
  j.setZero();
  j.topRows<3>().setIdentity();
  return true;
}

}  // namespace ct_lio::core
