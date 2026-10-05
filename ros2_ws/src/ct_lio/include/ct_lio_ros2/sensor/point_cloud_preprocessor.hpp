// point_cloud_preprocessor.hpp - 点云预处理(仅支持 ROS2 PointCloud2)
// 职责: PointCloud2 -> core::Point3D, 提取每点时间戳/alpha, 距离/跳点过滤
// 不做排序(点默认有序); 不做降采样(降采样移到消费线程)
#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ct_lio_ros2/core/frame_types.hpp"
#include "ct_lio_ros2/utils/eigen_types.hpp"
#include "ct_lio_ros2/utils/params.hpp"

namespace ct_lio::sensor {

/// PointCloud2 预处理: 只支持 sensor_msgs/PointCloud2
class PointCloudPreprocessor {
 public:
  explicit PointCloudPreprocessor(const PreprocessParams& params) : params_(params) {}

  /// 转换: PointCloud2 -> vector<core::Point3D>
  /// 提取每点时间戳(假设有序, 从 fields 里找 time/timestamp), 距离/盲区过滤
  bool Process(const sensor_msgs::msg::PointCloud2& msg, std::vector<core::Point3D>& out,
               double& timespan_out);

 private:
  PreprocessParams params_;
};

}  // namespace ct_lio::sensor
