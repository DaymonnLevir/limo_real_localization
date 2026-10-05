#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "ct_lio_ros2/sensor/point_cloud_preprocessor.hpp"

int main() {
  using sensor_msgs::msg::PointCloud2;
  using sensor_msgs::msg::PointField;
  auto field = [](const char* name, uint32_t offset, uint8_t type) {
    PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = type;
    f.count = 1;
    return f;
  };
  PointCloud2 msg;
  msg.width = 3;
  msg.height = 1;
  msg.point_step = 26;  // Actual Livox driver2 XYZRTL layout.
  msg.row_step = msg.width * msg.point_step;
  msg.fields = {field("x", 0, PointField::FLOAT32),
                field("y", 4, PointField::FLOAT32),
                field("z", 8, PointField::FLOAT32),
                field("intensity", 12, PointField::FLOAT32),
                field("tag", 16, PointField::UINT8),
                field("line", 17, PointField::UINT8),
                field("timestamp", 18, PointField::FLOAT64)};
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < 3; ++i) {
    const float x = 1.0f;
    const double t = 1.0e12 + static_cast<double>(i) * 5.0e7;
    std::memcpy(msg.data.data() + i * msg.point_step, &x, sizeof(x));
    std::memcpy(msg.data.data() + i * msg.point_step + 18, &t, sizeof(t));
  }
  ct_lio::PreprocessParams params;
  ct_lio::sensor::PointCloudPreprocessor preprocess(params);
  std::vector<ct_lio::core::Point3D> points;
  double span = 0.0;
  if (!preprocess.Process(msg, points, span)) return 1;
  if (points.size() != 3) return 2;
  if (std::fabs(span - 0.1) >= 1e-9) return 3;
  if (std::fabs(points[1].relative_time - 0.05) >= 1e-9) return 4;
  if (std::fabs(points[1].alpha_time - 0.5) >= 1e-9) return 5;
  msg.fields.pop_back();
  if (preprocess.Process(msg, points, span)) return 6;
  return 0;
}
