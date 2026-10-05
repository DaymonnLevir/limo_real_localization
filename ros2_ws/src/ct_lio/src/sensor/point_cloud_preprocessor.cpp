// point_cloud_preprocessor.cpp - 点云预处理实现
// 按 point_step 字节级解析 PointCloud2(不依赖 pcl::fromROSMsg), 兼容任意字段布局;
// 按 tag 字段过滤 livox 回波点, 将每点时间戳转为相对帧首时间。
#include "ct_lio_ros2/sensor/point_cloud_preprocessor.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace ct_lio::sensor {

using core::Point3D;

bool PointCloudPreprocessor::Process(const sensor_msgs::msg::PointCloud2& msg,
                                     std::vector<Point3D>& out, double& timespan_out) {
  out.clear();

  size_t point_step = msg.point_step;
  if (point_step == 0 || msg.data.empty() || msg.width == 0) return false;

  // 找字段偏移
  int off_x = -1, off_y = -1, off_z = -1, off_i = -1, off_t = -1, off_tag = -1;
  std::string time_field_name;
  uint8_t time_field_type = 0;
  for (const auto& f : msg.fields) {
    if (f.name == "x") off_x = static_cast<int>(f.offset);
    else if (f.name == "y") off_y = static_cast<int>(f.offset);
    else if (f.name == "z") off_z = static_cast<int>(f.offset);
    else if (f.name == "intensity" || f.name == "reflectivity") off_i = static_cast<int>(f.offset);
    else if (f.name == "time" || f.name == "timestamp" || f.name == "t") {
      off_t = static_cast<int>(f.offset);
      time_field_name = f.name;
      time_field_type = f.datatype;
    }
    else if (f.name == "tag") off_tag = static_cast<int>(f.offset);
  }
  // CT-LIO needs real per-point timing. Never invent a 0.1 s scan duration.
  if (off_x < 0 || off_y < 0 || off_z < 0 || off_t < 0 ||
      time_field_type != sensor_msgs::msg::PointField::FLOAT64 ||
      static_cast<size_t>(off_t) + sizeof(double) > point_step) return false;

  size_t n = msg.width * msg.height;
  out.reserve(n / static_cast<size_t>(params_.point_filter_num) + 1);
  const uint8_t* data = msg.data.data();

  // 首点时间戳作为相对时间基准
  // livox PointCloud2 的 timestamp 字段是绝对纳秒(float64), 其他雷达可能是相对秒;
  // 按首点量级判断: >1e6 视为纳秒量级, 否则视为秒量级。
  double first_t = 0.0;
  if (off_t >= 0) {
    memcpy(&first_t, data + static_cast<size_t>(off_t), sizeof(double));
  }
  // Livox driver2 calls the field "timestamp" and stores absolute nanoseconds.
  // Name-based detection also handles a frame whose first sensor timestamp is 0.
  const bool ts_in_ns = time_field_name == "timestamp" || std::fabs(first_t) > 1e6;

  double max_rel = 0.0;
  size_t idx = 0;
  for (size_t i = 0; i < n; ++i) {
    if (params_.point_filter_num > 1 && (idx % static_cast<size_t>(params_.point_filter_num) != 0)) {
      idx++;
      continue;
    }
    idx++;

    const uint8_t* p = data + i * point_step;
    float fx, fy, fz, fi = 0.0f;
    memcpy(&fx, p + static_cast<size_t>(off_x), sizeof(float));
    memcpy(&fy, p + static_cast<size_t>(off_y), sizeof(float));
    memcpy(&fz, p + static_cast<size_t>(off_z), sizeof(float));
    if (off_i >= 0) memcpy(&fi, p + static_cast<size_t>(off_i), sizeof(float));

    if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) continue;

    // livox tag 过滤: 只保留正常点(0x00)与反射率点(0x10)
    if (off_tag >= 0) {
      uint8_t tag = 0;
      memcpy(&tag, p + static_cast<size_t>(off_tag), sizeof(uint8_t));
      if (!(((tag & 0x30) == 0x10) || ((tag & 0x30) == 0x00))) continue;
    }

    double range_sq = static_cast<double>(fx) * fx + static_cast<double>(fy) * fy +
                      static_cast<double>(fz) * fz;
    if (range_sq > params_.max_range * params_.max_range || range_sq < params_.blind * params_.blind) continue;

    Point3D pt;
    pt.raw_point = Eigen::Vector3d(fx, fy, fz);
    pt.point = pt.raw_point;
    pt.intensity = fi;

    if (off_t >= 0) {
      double t = 0.0;
      memcpy(&t, p + static_cast<size_t>(off_t), sizeof(double));
      pt.relative_time = (t - first_t) / (ts_in_ns ? 1e9 : 1.0);
      if (pt.relative_time > max_rel) max_rel = pt.relative_time;
    }
    out.push_back(pt);
  }

  if (out.empty()) return false;

  if (max_rel <= 1e-6) return false;
  timespan_out = max_rel;
  for (auto& p : out) {
    p.alpha_time = timespan_out > 1e-6 ? p.relative_time / timespan_out : 0.0;
  }
  return true;
}

}  // namespace ct_lio::sensor
