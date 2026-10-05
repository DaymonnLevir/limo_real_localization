// ct_factors.cpp - CT 因子实现(完整移植自原版 lidarFactor.cpp)
// 各因子的残差定义与参数块布局见 ct_factors.hpp.
#include "ct_lio_ros2/core/ct_factors.hpp"

namespace ct_lio::core {

// ---------------------------------------------------------------------------
// 连续时间点到面因子: 残差与四个参数块的雅可比
// ---------------------------------------------------------------------------
bool CtPointToPlaneFactor::Evaluate(double const* const* parameters, double* residuals,
                                    double** jacobians) const {
  const Eigen::Vector3d tran_begin(parameters[0][0], parameters[0][1], parameters[0][2]);
  const Eigen::Vector3d tran_end(parameters[2][0], parameters[2][1], parameters[2][2]);
  const Eigen::Quaterniond rot_begin(parameters[1][3], parameters[1][0], parameters[1][1],
                                     parameters[1][2]);
  const Eigen::Quaterniond rot_end(parameters[3][3], parameters[3][0], parameters[3][1],
                                   parameters[3][2]);

  // 按 alpha 对 begin/end 位姿插值, 得到点所在时刻的连续时间位姿
  Eigen::Quaterniond rot_slerp = rot_begin.slerp(alpha_time_, rot_end);
  rot_slerp.normalize();
  Eigen::Vector3d tran_slerp = tran_begin * (1 - alpha_time_) + tran_end * alpha_time_;
  Eigen::Vector3d point_world = rot_slerp * raw_keypoint_ + tran_slerp;

  // 点到平面距离残差, 经 sqrt_info 与 weight 加权
  double distance = norm_vector_.dot(point_world) + norm_offset_;
  residuals[0] = sqrt_info_ * weight_ * distance;

  if (jacobians) {
    // 旋转雅可比公共部分: 平面法向在旋转后的叉乘矩阵作用在原始点上
    Eigen::Matrix<double, 1, 3> jacobian_rot_slerp =
        -norm_vector_.transpose() * rot_slerp.toRotationMatrix() *
        NumType::SkewSymmetric(raw_keypoint_) * weight_;

    // slerp 链式: 中间插值旋转与相对旋转, 用于对 begin/end 分别求偏导
    Eigen::Quaterniond rot_delta = rot_begin.inverse() * rot_end;
    Eigen::Quaterniond rot_identity(Eigen::Matrix3d::Identity());
    Eigen::Quaterniond rot_delta_slerp = rot_identity.slerp(alpha_time_, rot_delta);

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
      jacobian_tran_begin.setZero();
      jacobian_tran_begin.block<1, 3>(0, 0) = norm_vector_.transpose() * weight_ * (1 - alpha_time_);
      jacobian_tran_begin = sqrt_info_ * jacobian_tran_begin;
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[1]);
      jacobian_rot_begin.setZero();

      Eigen::Matrix<double, 3, 3> jacobian_slerp_begin =
          (rot_delta_slerp.toRotationMatrix()).transpose() *
          (Eigen::Matrix3d::Identity() -
           alpha_time_ * NumType::Qleft(rot_delta_slerp).bottomRightCorner<3, 3>() *
               (NumType::Qleft(rot_delta).bottomRightCorner<3, 3>()).inverse());

      jacobian_rot_begin.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_begin;
      jacobian_rot_begin = sqrt_info_ * jacobian_rot_begin;
    }
    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[2]);
      jacobian_tran_end.setZero();
      jacobian_tran_end.block<1, 3>(0, 0) = norm_vector_.transpose() * weight_ * alpha_time_;
      jacobian_tran_end = sqrt_info_ * jacobian_tran_end;
    }
    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_end(jacobians[3]);
      jacobian_rot_end.setZero();

      Eigen::Matrix<double, 3, 3> jacobian_slerp_end =
          alpha_time_ * NumType::Qright(rot_delta_slerp).bottomRightCorner<3, 3>() *
          (NumType::Qright(rot_delta).bottomRightCorner<3, 3>()).inverse();

      jacobian_rot_end.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_end;
      jacobian_rot_end = sqrt_info_ * jacobian_rot_end;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 位置一致性因子: 残差 = beta * (本帧 begin 位置 - 上一帧 end 位置)
// ---------------------------------------------------------------------------
bool LocationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals,
                                         double** jacobians) const {
  residuals[0] = beta_ * (parameters[0][0] - previous_location_(0, 0));
  residuals[1] = beta_ * (parameters[0][1] - previous_location_(1, 0));
  residuals[2] = beta_ * (parameters[0][2] - previous_location_(2, 0));

  if (jacobians && jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
    jacobian_tran_begin.setZero();
    jacobian_tran_begin(0, 0) = beta_;
    jacobian_tran_begin(1, 1) = beta_;
    jacobian_tran_begin(2, 2) = beta_;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 姿态一致性因子: 残差为两帧相对旋转的向量部分(beta 加权)
// ---------------------------------------------------------------------------
bool RotationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals,
                                         double** jacobians) const {
  Eigen::Quaterniond rot_cur(parameters[0][3], parameters[0][0], parameters[0][1],
                             parameters[0][2]);
  Eigen::Quaterniond q_et = previous_rotation_.inverse() * rot_cur;
  Eigen::Vector3d error = 2 * q_et.vec();

  residuals[0] = error[0] * beta_;
  residuals[1] = error[1] * beta_;
  residuals[2] = error[2] * beta_;

  if (jacobians && jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[0]);
    jacobian_rot_begin.setZero();
    jacobian_rot_begin.block<3, 3>(0, 0) =
        q_et.w() * Eigen::Matrix3d::Identity() + NumType::SkewSymmetric(q_et.vec());
    jacobian_rot_begin = jacobian_rot_begin * beta_;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 小速度因子: 残差 = beta * (本帧 begin 位置 - 本帧 end 位置)
// ---------------------------------------------------------------------------
bool SmallVelocityFactor::Evaluate(double const* const* parameters, double* residuals,
                                   double** jacobians) const {
  residuals[0] = beta_ * (parameters[0][0] - parameters[1][0]);
  residuals[1] = beta_ * (parameters[0][1] - parameters[1][1]);
  residuals[2] = beta_ * (parameters[0][2] - parameters[1][2]);

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
      jacobian_tran_begin.setZero();
      jacobian_tran_begin(0, 0) = beta_;
      jacobian_tran_begin(1, 1) = beta_;
      jacobian_tran_begin(2, 2) = beta_;
    }
    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[1]);
      jacobian_tran_end.setZero();
      jacobian_tran_end(0, 0) = -beta_;
      jacobian_tran_end(1, 1) = -beta_;
      jacobian_tran_end(2, 2) = -beta_;
    }
  }
  return true;
}

}  // namespace ct_lio::core
