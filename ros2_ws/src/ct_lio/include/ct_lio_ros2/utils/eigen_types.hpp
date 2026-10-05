// eigen_types.hpp - 常用 Eigen/Sophus 类型别名 (ct_lio 命名空间)
// 来源: 原 ct-lio src/common/eigen_types.h (slam-in-autodriving 书代码)
// 仅保留实际使用别名(零使用别名已清理)
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace ct_lio {

// ---------- 向量 ----------
using Vec3d = Eigen::Vector3d;
using Vec6d = Eigen::Matrix<double, 6, 1>;

// ---------- 矩阵 ----------
using Mat6d = Eigen::Matrix<double, 6, 6>;

// ---------- Sophus 李群 ----------
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

}  // namespace ct_lio
