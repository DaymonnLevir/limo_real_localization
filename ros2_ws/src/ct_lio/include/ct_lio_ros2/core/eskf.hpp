// eskf.hpp - 误差状态卡尔曼滤波器(ESKF)
//
// 职责: 以 IMU 递推为主、激光位姿为观测, 估计 18 维状态
//       [位置 p(3), 速度 v(3), 姿态 R(3), 陀螺零偏 bg(3),
//        加计零偏 ba(3), 重力 g(3)].
// 核心流程:
//   Predict      IMU 递推: 名义状态用常数加速度模型, 误差状态按
//                雅可比矩阵 F 线性化传播(对应书中式 3.47);
//   ObserveSE3   松耦合观测更新: 以激光位姿构造 6 维观测, 修正
//                误差状态并投影协方差.
// 来源: 移植自 slam-in-autodriving 一书的 ESKF 实现.
#pragma once

#include <cassert>

#include "ct_lio_ros2/core/imu_measurement.hpp"
#include "ct_lio_ros2/core/nav_state.hpp"
#include "ct_lio_ros2/utils/eigen_types.hpp"
#include "ct_lio_ros2/utils/math_utils.hpp"

namespace ct_lio::core {

/// 18 维 ESKF 滤波器, 状态排列顺序与 slam-in-autodriving 一书一致
template <typename S = double>
class ESKF {
 public:
  using SO3 = Sophus::SO3<S>;
  using VecT = Eigen::Matrix<S, 3, 1>;
  using Vec18T = Eigen::Matrix<S, 18, 1>;
  using Mat3T = Eigen::Matrix<S, 3, 3>;
  using MotionNoiseT = Eigen::Matrix<S, 18, 18>;
  using Mat18T = Eigen::Matrix<S, 18, 18>;
  using NavStateT = NavState<S>;

  /// 默认重力向量(对应参数 gravity_norm=9.81; 实际使用值由 IMU 初始化器在运行时覆盖)
  inline static const VecT kDefaultGravity = VecT(0, 0, -9.81);

  // ---------------------------------------------------------------------
  // 滤波器配置
  // ---------------------------------------------------------------------
  struct Options {
    Options() = default;
    double imu_dt = 0.01;
    double gyro_var = 1e-5;
    double acce_var = 1e-2;
    double bias_gyro_var = 1e-6;
    double bias_acce_var = 1e-4;
    bool update_bias_gyro = true;
    bool update_bias_acce = true;
  };

  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  /// 设置初始条件(零偏与重力), 由 IMU 初始化器在启动阶段调用
  void SetInitialConditions(Options options, const VecT& init_bg, const VecT& init_ba,
                            const VecT& gravity = kDefaultGravity) {
    BuildNoise(options);
    options_ = options;
    bg_ = init_bg;
    ba_ = init_ba;
    // 重力对齐: 用 body 系实测重力方向构造初始姿态 R0, 使 world Z 竖到重力反方向
    // gravity 由 ImuInitializer 按 gravity_norm 参数计算(大小=gravity_norm), 方向为 body 系实测
    const double g_norm = gravity.norm();  // 重力大小取配置值(来自 yaml gravity_norm), 不写死
    Eigen::Quaterniond q_align = Eigen::Quaterniond::Identity();
    if (g_norm > 1e-6) {
      // 求 R0 使 R0 * (gravity/g_norm) = [0,0,-1]: body系重力方向 -> world 重力反方向
      q_align = Eigen::Quaterniond::FromTwoVectors(gravity / g_norm, VecT(0, 0, -1));
      q_align.normalize();
    }
    R_ = SO3(q_align);            // 初始姿态 = 重力对齐(退化时保持 Identity, 等价原行为)
    g_ = VecT(0, 0, -g_norm);     // world 系重力, 大小=gravity_norm
    cov_ = Mat18T::Identity() * 1e-4;
  }

  /// IMU 递推一步: 名义状态与误差状态同步传播
  bool Predict(const ImuMeasurement& imu);

  /// 用 SE3 观测更新(松耦合核心: 激光位姿作为观测)
  bool ObserveSE3(const SE3& pose, double trans_noise = 0.1,
                  double ang_noise = 1.0 * kDEG2RAD);
  bool ObserveSE3(const SE3& pose, const Vec6d noise);

  // ---------------------------------------------------------------------
  // 状态访问
  // ---------------------------------------------------------------------
  /// 获取名义状态(时间戳 + 位置/速度/姿态/零偏)
  NavStateT GetNominalState() const { return NavStateT(current_time_, R_, p_, v_, bg_, ba_); }
  /// 名义速度
  VecT GetNominalVel() const { return v_; }

 private:
  // ---------------------------------------------------------------------
  // 内部实现
  // ---------------------------------------------------------------------
  void BuildNoise(const Options& options) {
    double ev = options.acce_var;
    double et = options.gyro_var;
    double eg = options.bias_gyro_var;
    double ea = options.bias_acce_var;

    // 与原版一致: 噪声直接取标准差原值(不再平方), 位置块噪声为 0
    double ev2 = ev;
    double et2 = et;
    double eg2 = eg;
    double ea2 = ea;

    // 过程噪声按状态块设置: 位置块(前 3)=0, 速度=acce_var,
    // 姿态=gyro_var, 陀螺零偏=bias_gyro_var, 加计零偏=bias_acce_var, 重力块(后 3)=0
    Q_.setZero();
    Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;
  }

  /// 更新名义状态并清零误差状态(同时完成协方差投影)
  void UpdateAndReset() {
    p_ += dx_.template block<3, 1>(0, 0);
    v_ += dx_.template block<3, 1>(3, 0);
    R_ = R_ * Sophus::SO3<S>::exp(dx_.template block<3, 1>(6, 0));

    if (options_.update_bias_gyro) bg_ += dx_.template block<3, 1>(9, 0);
    if (options_.update_bias_acce) ba_ += dx_.template block<3, 1>(12, 0);

    g_ += dx_.template block<3, 1>(15, 0);

    ProjectCov();  // 对协方差矩阵投影, 对应书中式(3.59)
    dx_.setZero();
  }

  /// 对协方差矩阵投影: 对 SO3 误差作二阶修正(书中式 3.59)
  void ProjectCov() {
    Mat18T J = Mat18T::Identity();
    J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
    cov_ = J * cov_ * J.transpose();
  }

  // ---------------------------------------------------------------------
  // 状态与协方差成员
  // ---------------------------------------------------------------------
  Options options_;
  VecT p_ = VecT::Zero();   // 位置
  VecT v_ = VecT::Zero();   // 速度
  SO3 R_;                   // 姿态
  VecT bg_ = VecT::Zero();  // 陀螺零偏
  VecT ba_ = VecT::Zero();  // 加计零偏
  VecT g_ = kDefaultGravity;  // 重力(默认 -Z 方向, 与 gravity_norm 参数一致)
  Vec18T dx_ = Vec18T::Zero();  // 误差状态增量
  Mat18T cov_ = Mat18T::Identity() * 1e-4;  // 协方差(SetInitialConditions 会重置为 1e-4)
  MotionNoiseT Q_ = MotionNoiseT::Zero();   // 过程噪声矩阵
  double current_time_ = 0.0;
};

template <typename S>
bool ESKF<S>::Predict(const ImuMeasurement& imu) {
  assert(imu.timestamp >= current_time_);

  double dt = imu.timestamp - current_time_;
  if (dt > (5 * options_.imu_dt) || dt < 0) {
    // 时间间隔异常(可能是第一帧 IMU 数据), 跳过本次递推
    current_time_ = imu.timestamp;
    return false;
  }

  // 名义状态递推(常数加速度模型)
  VecT new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
  VecT new_v = v_ + R_ * (imu.acce - ba_) * dt + g_ * dt;
  SO3 new_R = R_ * SO3::exp((imu.gyro - bg_) * dt);

  R_ = new_R;
  v_ = new_v;
  p_ = new_p;

  // 误差状态递推: 构造雅可比矩阵 F(对应书中式 3.47)
  Mat18T F = Mat18T::Identity();
  F.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;                        // p 对 v
  F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce - ba_) * dt;  // v 对 theta
  F.template block<3, 3>(3, 12) = -R_.matrix() * dt;                            // v 对 ba
  F.template block<3, 3>(3, 15) = Mat3T::Identity() * dt;                       // v 对 g
  F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro - bg_) * dt).matrix();     // theta 对 theta
  F.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;                       // theta 对 bg

  dx_ = F * dx_;
  cov_ = F * cov_.eval() * F.transpose() + Q_;
  current_time_ = imu.timestamp;
  return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
  Vec6d noise;
  noise << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
  return ObserveSE3(pose, noise);
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, const Vec6d noise) {
  // 观测位置与姿态: 观测矩阵 H 为 6x18, 对应 p 块与 R 块
  Eigen::Matrix<S, 6, 18> H = Eigen::Matrix<S, 6, 18>::Zero();
  H.template block<3, 3>(0, 0) = Mat3T::Identity();
  H.template block<3, 3>(3, 6) = Mat3T::Identity();

  Mat6d V = noise.asDiagonal();
  Eigen::Matrix<S, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();

  Vec6d innov = Vec6d::Zero();
  innov.template head<3>() = (pose.translation() - p_);
  innov.template tail<3>() = (R_.inverse() * pose.so3()).log();

  dx_ = K * innov;
  cov_ = (Mat18T::Identity() - K * H) * cov_;
  UpdateAndReset();
  return true;
}

}  // namespace ct_lio::core

// ---------------------------------------------------------------------------
// 常用别名: 与原版 CT-LIO 的 ESKFD 类型保持一致
// ---------------------------------------------------------------------------
namespace ct_lio::core {
using ESKFD = ESKF<double>;
}  // namespace ct_lio::core
