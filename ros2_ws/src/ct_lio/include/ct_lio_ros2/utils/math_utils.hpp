// math_utils.hpp - 数学工具 (ct_lio 命名空间版)
// 来源: 原 ct-lio src/common/math_utils.h (slam-in-autodriving 书代码)
#pragma once

#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "ct_lio_ros2/utils/eigen_types.hpp"

namespace ct_lio {

constexpr double kDEG2RAD = M_PI / 180.0;

/// 把数组转成 Eigen 向量(长度≠3 抛异常, 由调用方校验参数转失败)
template <typename T>
inline Eigen::Matrix<T, 3, 1> vecFromArray(const std::vector<T>& v) {
  if (v.size() != 3) {
    throw std::invalid_argument("vecFromArray: expected size 3, got " + std::to_string(v.size()));
  }
  return Eigen::Matrix<T, 3, 1>(v[0], v[1], v[2]);
}

/// 把 9 个数(行主序)转成 3x3 矩阵(长度≠9 抛异常)
template <typename T>
inline Eigen::Matrix<T, 3, 3> matFromArray(const std::vector<T>& v) {
  if (v.size() != 9) {
    throw std::invalid_argument("matFromArray: expected size 9, got " + std::to_string(v.size()));
  }
  Eigen::Matrix<T, 3, 3> m;
  for (int i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
  return m;
}

/// 计算均值与协方差对角元
/// 方差归一化用 N-1(无偏估计), 与原版 ct-lio math_utils.h 完全一致
template <typename C, typename F>
inline void computeMeanAndCovDiag(const C& data, Eigen::Vector3d& mean,
                                  Eigen::Vector3d& cov_diag, F&& getter) {
  mean.setZero();
  if (data.size() < 2) {
    cov_diag.setZero();
    return;
  }
  for (const auto& d : data) mean += getter(d);
  mean /= static_cast<double>(data.size());

  cov_diag.setZero();
  for (const auto& d : data) {
    Eigen::Vector3d diff = getter(d) - mean;
    cov_diag += diff.cwiseProduct(diff);
  }
  cov_diag /= static_cast<double>(data.size() - 1);  // N-1 无偏(原版一致)
}

}  // namespace ct_lio
