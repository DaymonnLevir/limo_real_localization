// lio_node.hpp - ROS2 节点类(节点层): 参数加载/订阅/发布/TF/消费线程/地图保存
#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ct_lio_ros2/core/lidar_odometry.hpp"
#include "ct_lio_ros2/sensor/point_cloud_preprocessor.hpp"
#include "ct_lio_ros2/utils/params.hpp"

namespace ct_lio {

/// ROS2 节点: 参数加载 + 订阅 + 发布 + TF + 消费线程
class LioNode : public rclcpp::Node {
 public:
  LioNode();
  ~LioNode() override;

  /// 启动是否成功(核心 Initialize + 订阅初始化完成); 失败时 main 应退出非零
  bool IsInitialized() const { return initialized_; }

 private:
  // 回调
  void LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // 输出回调(从核心注入)
  void OnPose(const SE3& pose, double stamp);
  void OnCloud(const std::vector<core::Point3D>& points, double stamp);
  // 地图保存(增量分片 + 退出融合)
  void OnMapChunk(const std::vector<core::Point3D>& points, double stamp);
  void FinalizeMapSave();

  // 参数加载(先声明默认值, 再由参数文件/launch 覆盖)
  void LoadParameters();

  // 时间戳域校准: 自动取"时钟-数据"差值, 输出时间戳 = 数据时间 + 偏移
  void CalibrateTimeOffset(double data_time);

  // 核心 + 预处理
  std::unique_ptr<core::LidarOdometry> odometry_;
  std::unique_ptr<sensor::PointCloudPreprocessor> preprocessor_;

  // IMU 饱和外推用: 上次有效测量
  std::shared_ptr<core::ImuMeasurement> last_valid_imu_;  // 上次有效 IMU(饱和外推用)
  double last_imu_time_ = -1.0;                      // 上次 IMU 时间戳(回退检测用)

  // 参数
  CommonParams common_params_;
  PreprocessParams preprocess_params_;
  OdometryParams odometry_params_;
  SaveMapParams save_map_params_;

  // 发布器
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  // 订阅
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 消费线程
  std::thread processing_thread_;
  std::atomic<bool> running_{false};
  bool initialized_ = false;   // 构造函数完成后置 true(供 main 判断启动成败)

  // 时间戳域校准
  double time_offset_ = 0.0;        // 生效偏移(手动或自动校准结果)
  bool offset_manual_ = false;      // 用户手动指定了 time_offset(跳过自动校准)
  bool offset_calibrated_ = false;  // 自动校准已完成

  // 轨迹
  nav_msgs::msg::Path path_msg_;
  std::mutex path_mutex_;
};

}  // namespace ct_lio
