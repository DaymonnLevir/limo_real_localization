// ct_factors.hpp - Ceres 因子(CT-ICP 核心)
//
// 职责: 为连续时间优化提供代价函数.
// 当前采用固定 CT 模式, 由两类因子构成:
//   CtPointToPlaneFactor   连续时间点到面因子(帧内位姿按时间插值);
//   一致性正则因子         约束本帧 begin 与上帧 end 的位姿一致.
// 实现完整移植自原版 ct-lio 的 lidarFactor.h/cpp, 含基于四元数
// Qleft/Qright 矩阵的解析链式雅可比推导.
#pragma once

#include <ceres/ceres.h>

#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio::core {

/// 四元数运算工具(对应原版 CT-LIO 的 numType; 原版 deltaQ 已内联进 RotationParameterization::Plus)
struct NumType {
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(
      const Eigen::MatrixBase<Derived>& mat) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
    mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
        mat(2), typename Derived::Scalar(0), -mat(0),
        -mat(1), mat(0), typename Derived::Scalar(0);
    return mat_skew;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(
      const Eigen::QuaternionBase<Derived>& q) {
    Eigen::Quaternion<typename Derived::Scalar> qq = q;
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(),
    ans.template block<3, 3>(1, 1) =
        qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() +
        SkewSymmetric(qq.vec());
    return ans;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(
      const Eigen::QuaternionBase<Derived>& p) {
    Eigen::Quaternion<typename Derived::Scalar> pp = p;
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(),
    ans.template block<3, 3>(1, 1) =
        pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() -
        SkewSymmetric(pp.vec());
    return ans;
  }
};

// ---------------------------------------------------------------------------
// 连续时间点到面因子 (CT-P2P): 一帧内两个位姿(begin/end), 点按 alpha 插值
// 残差 = n^T * (R(alpha)*p + t(alpha)) + offset
// 参数块: [begin_t(3), begin_quat(4), end_t(3), end_quat(4)]
// 雅可比: 完整 slerp 链式(原版 CTLidarPlaneNormFactor)
// ---------------------------------------------------------------------------
class CtPointToPlaneFactor : public ceres::SizedCostFunction<1, 3, 4, 3, 4> {
 public:
  // 原版使用静态成员存储数据, 进程内多实例会互相覆盖;
  // 这里改为实例成员, 构造时由调用方传入
  CtPointToPlaneFactor(const Eigen::Vector3d& raw_keypoint, const Eigen::Vector3d& norm_vector,
                       double norm_offset, double alpha_time, const Eigen::Vector3d& t_il,
                       const Eigen::Quaterniond& q_il, double sqrt_info, double weight = 1.0)
      : norm_vector_(norm_vector), norm_offset_(norm_offset), alpha_time_(alpha_time),
        sqrt_info_(sqrt_info), weight_(weight) {
    raw_keypoint_ = q_il * raw_keypoint + t_il;  // 点转到 IMU 系
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

  Eigen::Vector3d raw_keypoint_;
  Eigen::Vector3d norm_vector_;
  double norm_offset_;
  double alpha_time_;
  double sqrt_info_;
  double weight_;
};

// ---------------------------------------------------------------------------
// 位置一致性因子: 约束本帧 begin 位置与上一帧 end 位置一致
// ---------------------------------------------------------------------------
class LocationConsistencyFactor : public ceres::SizedCostFunction<3, 3> {
 public:
  LocationConsistencyFactor(const Eigen::Vector3d& previous_location, double beta)
      : previous_location_(previous_location), beta_(beta) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

  Eigen::Vector3d previous_location_;
  double beta_;
};

// ---------------------------------------------------------------------------
// 姿态一致性因子: 约束本帧 begin 姿态与上一帧 end 姿态一致
// ---------------------------------------------------------------------------
class RotationConsistencyFactor : public ceres::SizedCostFunction<3, 4> {
 public:
  RotationConsistencyFactor(const Eigen::Quaterniond& previous_rotation, double beta)
      : previous_rotation_(previous_rotation), beta_(beta) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

  Eigen::Quaterniond previous_rotation_;
  double beta_;
};

// ---------------------------------------------------------------------------
// 小速度因子: 约束帧内位移足够小(默认关闭)
// ---------------------------------------------------------------------------
class SmallVelocityFactor : public ceres::SizedCostFunction<3, 3, 3> {
 public:
  explicit SmallVelocityFactor(double beta) : beta_(beta) {}
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override;

  double beta_;
};

}  // namespace ct_lio::core
