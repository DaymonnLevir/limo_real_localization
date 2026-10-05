// lio_node.cpp - ROS2 节点实现(节点层)
// 职责: 参数加载、发布/订阅器创建、输出回调注入与消费线程管理, 桥接核心算法与 ROS 通信; 另负责地图保存
#include "ct_lio_ros2/ros/lio_node.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


namespace ct_lio {

LioNode::LioNode() : Node("ct_lio_node") {
  // -------------------- 参数加载 --------------------
  // 声明默认值, 再由参数文件/launch 覆盖(不写死)
  LoadParameters();

  // -------------------- 核心初始化 --------------------
  odometry_ = std::make_unique<core::LidarOdometry>();
  if (!odometry_->Initialize(odometry_params_)) {
    RCLCPP_ERROR(get_logger(), "LidarOdometry init failed");
    return;
  }

  // -------------------- 输出发布(回调注入) --------------------
  // 具名函数, 不匿名 lambda
  odometry_->SetPoseCallback([this](const SE3& pose, double stamp) { OnPose(pose, stamp); });
  odometry_->SetCloudCallback([this](const std::vector<core::Point3D>& points, double stamp) {
    OnCloud(points, stamp);
  });
  // 地图保存回调(增量分片写盘)
  odometry_->SetMapChunkCallback([this](const std::vector<core::Point3D>& points, double stamp) {
    OnMapChunk(points, stamp);
  });
  // 保存配置生效(默认关闭, 零开销)
  odometry_->SetMapSaveParams(save_map_params_);

  // 预处理单元(紧随里程计创建, 供订阅回调使用)
  preprocessor_ = std::make_unique<sensor::PointCloudPreprocessor>(preprocess_params_);

  // -------------------- 发布器(话题名可配) --------------------
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(common_params_.odom_topic, 10);
  if (common_params_.publish_path) {
    path_pub_ = create_publisher<nav_msgs::msg::Path>(common_params_.path_topic, 5);
  }
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(common_params_.scan_topic, 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // -------------------- 订阅 --------------------
  // 当前 Orin 的 Livox 驱动发布 RELIABLE 大点云和 IMU。实测 Fast DDS 下 Best Effort
  // 读者无法稳定收到分片 PointCloud2，必须按实际发布端使用 RELIABLE；若未来驱动改为
  // Best Effort，应同步将此处改回 Best Effort，不能依赖“理论上可协商”。
  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<int>(common_params_.lidar_freq_hz * 3)))
                       .reliable()
                       .durability_volatile();
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<int>(common_params_.imu_freq_hz * 3)))
                     .reliable()
                     .durability_volatile();

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      common_params_.lidar_topic, lidar_qos,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { LidarCallback(msg); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      common_params_.imu_topic, imu_qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { ImuCallback(msg); });

  // -------------------- 消费线程(回调只入队, 算法在独立线程) --------------------
  running_ = true;
  processing_thread_ = std::thread([this] {
    odometry_->Run();
    running_ = false;
  });

  // 全部初始化完成, 标记启动成功(供 main 判断; 前面任一失败分支 return 时保持 false)
  initialized_ = true;

  RCLCPP_INFO(get_logger(), "ct_lio_node started: lidar=%s imu=%s odom=%s",
              common_params_.lidar_topic.c_str(), common_params_.imu_topic.c_str(),
              common_params_.odom_topic.c_str());
}

// -------------------- 析构与退出收尾 --------------------
LioNode::~LioNode() {
  // 先唤醒消费线程再 join, 否则其永久阻塞在条件变量上, Ctrl+C 时挂死
  if (odometry_) odometry_->Stop();
  if (processing_thread_.joinable()) processing_thread_.join();
  // 地图保存收尾: 最后一段增量分片 + 融合/体素滤波/清理(启用时)
  if (odometry_ && save_map_params_.enabled) {
    RCLCPP_INFO(get_logger(), "[map_save] Ctrl+C exit, finalizing map save...");
    odometry_->SaveMapChunk(0.0);
    FinalizeMapSave();
    RCLCPP_INFO(get_logger(), "[map_save] all done, chunks cleaned, keep fused map only");
  }
}

// -------------------- 参数加载 --------------------
// 集中声明参数默认值, 由参数文件/launch 覆盖; 构造时先于核心初始化调用, 保证参数就绪
void LioNode::LoadParameters() {
  // 公共参数: 话题/坐标系/频率/手动时间偏移
  common_params_.lidar_topic = declare_parameter<std::string>("lidar_topic", common_params_.lidar_topic);
  common_params_.imu_topic = declare_parameter<std::string>("imu_topic", common_params_.imu_topic);
  common_params_.odom_topic = declare_parameter<std::string>("odom_topic", common_params_.odom_topic);
  common_params_.path_topic = declare_parameter<std::string>("path_topic", common_params_.path_topic);
  common_params_.scan_topic = declare_parameter<std::string>("scan_topic", common_params_.scan_topic);
  common_params_.publish_path = declare_parameter<bool>("publish_path", common_params_.publish_path);
  common_params_.lidar_freq_hz = declare_parameter<double>("lidar_freq_hz", common_params_.lidar_freq_hz);
  common_params_.imu_freq_hz = declare_parameter<double>("imu_freq_hz", common_params_.imu_freq_hz);
  common_params_.world_frame = declare_parameter<std::string>("world_frame", common_params_.world_frame);
  common_params_.body_frame = declare_parameter<std::string>("body_frame", common_params_.body_frame);
  common_params_.time_offset =
      declare_parameter<double>("time_offset", common_params_.time_offset);
  if (common_params_.time_offset != 0.0) {
    time_offset_ = common_params_.time_offset;
    offset_manual_ = true;
  }

  // 预处理参数: 抽稀/盲区/量程
  preprocess_params_.point_filter_num =
      declare_parameter<int>("preprocess.point_filter_num", preprocess_params_.point_filter_num);
  preprocess_params_.blind = declare_parameter<double>("preprocess.blind", preprocess_params_.blind);
  preprocess_params_.max_range =
      declare_parameter<double>("preprocess.max_range", preprocess_params_.max_range);

  // 里程计参数(按子模块分组)
  // 外参与传感器时间延迟
odometry_params_.extrinsic_t =
      declare_parameter<std::vector<double>>("odometry.extrinsic_t", odometry_params_.extrinsic_t);
  odometry_params_.extrinsic_r =
      declare_parameter<std::vector<double>>("odometry.extrinsic_r", odometry_params_.extrinsic_r);
  odometry_params_.delay_time =
      declare_parameter<double>("odometry.delay_time", odometry_params_.delay_time);
  // CT-ICP 匹配: 分辨率/迭代/降采样/Ceres
odometry_params_.surf_res =
      declare_parameter<double>("odometry.surf_res", odometry_params_.surf_res);
  odometry_params_.max_num_iteration =
      declare_parameter<int>("odometry.max_num_iteration", odometry_params_.max_num_iteration);
  odometry_params_.downsample_size = declare_parameter<double>("odometry.downsample_size",
                                                               odometry_params_.downsample_size);
  odometry_params_.ceres_max_num_iterations = declare_parameter<int>(
      "odometry.ceres_max_num_iterations", odometry_params_.ceres_max_num_iterations);
  odometry_params_.ceres_num_threads = declare_parameter<int>(
      "odometry.ceres_num_threads", odometry_params_.ceres_num_threads);
  odometry_params_.ceres_huber_scale = declare_parameter<double>(
      "odometry.ceres_huber_scale", odometry_params_.ceres_huber_scale);
  // 体素地图: 体素尺寸/点数上限/容量/最大距离
odometry_params_.size_voxel_map =
      declare_parameter<double>("odometry.size_voxel_map", odometry_params_.size_voxel_map);
  odometry_params_.min_distance_points =
      declare_parameter<double>("odometry.min_distance_points", odometry_params_.min_distance_points);
  odometry_params_.max_num_points_in_voxel = declare_parameter<int>(
      "odometry.max_num_points_in_voxel", odometry_params_.max_num_points_in_voxel);
  odometry_params_.capacity =
      declare_parameter<int>("odometry.capacity", odometry_params_.capacity);
  odometry_params_.max_distance =
      declare_parameter<double>("odometry.max_distance", odometry_params_.max_distance);
  // 邻域搜索: 邻域范围/平面性/占用率
odometry_params_.voxel_neighborhood =
      declare_parameter<int>("odometry.voxel_neighborhood", odometry_params_.voxel_neighborhood);
  odometry_params_.max_number_neighbors =
      declare_parameter<int>("odometry.max_number_neighbors", odometry_params_.max_number_neighbors);
  odometry_params_.min_number_neighbors =
      declare_parameter<int>("odometry.min_number_neighbors", odometry_params_.min_number_neighbors);
  odometry_params_.power_planarity =
      declare_parameter<double>("odometry.power_planarity", odometry_params_.power_planarity);
  odometry_params_.threshold_voxel_occupancy = declare_parameter<int>(
      "odometry.threshold_voxel_occupancy", odometry_params_.threshold_voxel_occupancy);
  odometry_params_.num_closest_neighbors = declare_parameter<int>(
      "odometry.num_closest_neighbors", odometry_params_.num_closest_neighbors);
  odometry_params_.max_num_residuals = declare_parameter<int>(
      "odometry.max_num_residuals", odometry_params_.max_num_residuals);
  // 残差/权重: 面距离阈值/权重系数/采样率
odometry_params_.max_dist_to_plane_icp = declare_parameter<double>(
      "odometry.max_dist_to_plane_icp", odometry_params_.max_dist_to_plane_icp);
  odometry_params_.weight_alpha =
      declare_parameter<double>("odometry.weight_alpha", odometry_params_.weight_alpha);
  odometry_params_.weight_neighborhood =
      declare_parameter<double>("odometry.weight_neighborhood", odometry_params_.weight_neighborhood);
  odometry_params_.init_num_frames =
      declare_parameter<int>("odometry.init_num_frames", odometry_params_.init_num_frames);
  odometry_params_.sampling_rate =
      declare_parameter<double>("odometry.sampling_rate", odometry_params_.sampling_rate);
  // 一致性正则: 位姿一致性/小速度项
odometry_params_.beta_location_consistency = declare_parameter<double>(
      "odometry.beta_location_consistency", odometry_params_.beta_location_consistency);
  odometry_params_.beta_orientation_consistency = declare_parameter<double>(
      "odometry.beta_orientation_consistency", odometry_params_.beta_orientation_consistency);
  odometry_params_.beta_small_velocity = declare_parameter<double>(
      "odometry.beta_small_velocity", odometry_params_.beta_small_velocity);
  // 收敛判定: 平移/旋转阈值
odometry_params_.thres_translation_norm = declare_parameter<double>(
      "odometry.thres_translation_norm", odometry_params_.thres_translation_norm);
  odometry_params_.thres_orientation_norm = declare_parameter<double>(
      "odometry.thres_orientation_norm", odometry_params_.thres_orientation_norm);
  // ESKF 噪声: 激光点协方差/名义速度噪声
odometry_params_.laser_point_cov =
      declare_parameter<double>("eskf.laser_point_cov", odometry_params_.laser_point_cov);
  odometry_params_.odom_trans_noise =
      declare_parameter<double>("eskf.odom_trans_noise", odometry_params_.odom_trans_noise);
  odometry_params_.odom_ang_noise =
      declare_parameter<double>("eskf.odom_ang_noise", odometry_params_.odom_ang_noise);
  // IMU 初始化: 模式/重力/静止判定
odometry_params_.init_mode =
      declare_parameter<std::string>("eskf.init_mode", odometry_params_.init_mode);
  odometry_params_.gravity_norm =
      declare_parameter<double>("eskf.gravity_norm", odometry_params_.gravity_norm);
  odometry_params_.init_time_seconds =
      declare_parameter<double>("eskf.init_time_seconds", odometry_params_.init_time_seconds);
  odometry_params_.dynamic_min_imu_count = declare_parameter<int>(
      "eskf.dynamic_min_imu_count", odometry_params_.dynamic_min_imu_count);
  odometry_params_.max_static_gyro_var =
      declare_parameter<double>("eskf.max_static_gyro_var", odometry_params_.max_static_gyro_var);
  odometry_params_.max_static_acce_var =
      declare_parameter<double>("eskf.max_static_acce_var", odometry_params_.max_static_acce_var);
  odometry_params_.imu_dt =
      declare_parameter<double>("eskf.imu_dt", odometry_params_.imu_dt);
  odometry_params_.gyro_var =
      declare_parameter<double>("eskf.gyro_var", odometry_params_.gyro_var);
  odometry_params_.acce_var =
      declare_parameter<double>("eskf.acce_var", odometry_params_.acce_var);
  odometry_params_.bias_gyro_var =
      declare_parameter<double>("eskf.bias_gyro_var", odometry_params_.bias_gyro_var);
  odometry_params_.bias_acce_var =
      declare_parameter<double>("eskf.bias_acce_var", odometry_params_.bias_acce_var);
  // IMU 饱和检测: 阈值/保持策略
  odometry_params_.check_satu =
      declare_parameter<bool>("eskf.check_satu", odometry_params_.check_satu);
  odometry_params_.satu_gyro =
      declare_parameter<double>("eskf.satu_gyro", odometry_params_.satu_gyro);
  odometry_params_.satu_acc =
      declare_parameter<double>("eskf.satu_acc", odometry_params_.satu_acc);
  odometry_params_.satu_hold =
      declare_parameter<bool>("eskf.satu_hold", odometry_params_.satu_hold);
  odometry_params_.satu_margin =
      declare_parameter<double>("eskf.satu_margin", odometry_params_.satu_margin);
  odometry_params_.satu_freeze_bias =
      declare_parameter<bool>("eskf.satu_freeze_bias", odometry_params_.satu_freeze_bias);

  // 地图保存: 增量分片 + 退出融合
  save_map_params_.enabled = declare_parameter<bool>("save_map.enabled", save_map_params_.enabled);
  save_map_params_.save_interval_frames = declare_parameter<int>(
      "save_map.save_interval_frames", save_map_params_.save_interval_frames);
  save_map_params_.voxel_filter_size = declare_parameter<double>(
      "save_map.voxel_filter_size", save_map_params_.voxel_filter_size);
  save_map_params_.save_dir = declare_parameter<std::string>("save_map.save_dir",
                                                              save_map_params_.save_dir);

  // 地图保存目录不存在则自动创建(防御式: 避免 PCD 写盘抛 pcl::IOException 崩溃)
  if (save_map_params_.enabled) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::create_directories(save_map_params_.save_dir, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "save dir create failed (%s): %s, map save disabled",
                  save_map_params_.save_dir.c_str(), ec.message().c_str());
      save_map_params_.enabled = false;
    } else {
      RCLCPP_INFO(get_logger(), "map save dir ready: %s", save_map_params_.save_dir.c_str());
    }
  }
}

// -------------------- 订阅回调 --------------------
// 回调只做数据转换与入队, 不做重计算, 算法在消费线程执行
void LioNode::LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // 转换 + 入队(轻量, 不做降采样/sort)
  std::vector<core::Point3D> cloud_out;
  // 初值仅占位: 实际帧跨度由 preprocessor 内部计算后输出(点云时间戳可得时)
  double timespan = 0.0;
  if (!preprocessor_->Process(*msg, cloud_out, timespan)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "lidar preprocess failed");
    return;
  }
  double begin_time =
      static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  odometry_->PushLidarFrame(std::move(cloud_out), begin_time, timespan);
}

void LioNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  double t = static_cast<double>(msg->header.stamp.sec) +
             static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  // 时间回退检测: bag 循环回退时重置上次有效值, 防止用"未来"IMU 数据外推
  if (last_imu_time_ > 0.0 && t < last_imu_time_) {
    last_valid_imu_ = nullptr;
  }
  last_imu_time_ = t;
  // livox IMU 加速度单位是 g, 需乘重力加速度转 m/s^2(重力值由 gravity_norm 参数配置)
  Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z);
  Eigen::Vector3d acce(msg->linear_acceleration.x * odometry_params_.gravity_norm,
                       msg->linear_acceleration.y * odometry_params_.gravity_norm,
                       msg->linear_acceleration.z * odometry_params_.gravity_norm);

  // ---- IMU 饱和检测(检测层) ----
  // 阈值按用户 IMU 量程配置(默认 livox 内置: gyro≈2005°/s, acc=3g)
  // 按轴独立判定/替换: 任一轴超阈值只替换该轴, 保留其他有效轴的真实读数
  bool saturated = false;
  if (odometry_params_.check_satu) {
    // ×satu_margin(默认0.99)留 1% 余量防浮点误触发(参考 Point-LIO)
    Eigen::Vector3d gyro_hold = gyro;
    Eigen::Vector3d acce_hold = acce;
    for (int i = 0; i < 3; ++i) {
      if (std::fabs(gyro(i)) > odometry_params_.satu_margin * odometry_params_.satu_gyro) {
        saturated = true;
        if (odometry_params_.satu_hold && last_valid_imu_) gyro_hold(i) = last_valid_imu_->gyro(i);
      }
      if (std::fabs(acce(i)) > odometry_params_.satu_margin * odometry_params_.satu_acc) {
        saturated = true;
        if (odometry_params_.satu_hold && last_valid_imu_) acce_hold(i) = last_valid_imu_->acce(i);
      }
    }
    // 首帧饱和时无上次有效值可保持, 削顶值直接入队并告警, 至少日志可见
    if (saturated && odometry_params_.satu_hold && !last_valid_imu_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "IMU saturated but no valid previous sample to hold; raw saturated "
                           "value will be used");
    }
    gyro = gyro_hold;
    acce = acce_hold;
  }

  auto imu = std::make_shared<core::ImuMeasurement>(t, gyro, acce);
  imu->saturated = saturated;
  if (!saturated) {
    last_valid_imu_ = imu;  // 记录上次有效值(供饱和外推)
  }
  odometry_->PushImu(imu);
}

// -------------------- 时间戳域校准 --------------------
void LioNode::CalibrateTimeOffset(double data_time) {
  // 自动校准: 首帧输出时取"时钟-数据"差值作为恒定偏移, 之后输出时间戳全部抬到时钟域。
  // 适用: mid360 驱动把设备 uptime 当时间戳(bag 回放或真机都能自动对齐)。
  // 偏移允许为负：设备时钟既可能落后，也可能领先主机时钟。
  // 若把负值截成 0，TF 会被发布到未来，message_filter 将持续丢弃激光数据。
  if (offset_manual_ || offset_calibrated_) return;
  double clock_now = get_clock()->now().seconds();
  time_offset_ = clock_now - data_time;
  offset_calibrated_ = true;
  RCLCPP_INFO(get_logger(), "data time domain offset calibrated: %.6f s (clock=%.3f data=%.3f)",
              time_offset_, clock_now, data_time);
}

// -------------------- 输出发布(回调注入) --------------------
// 里程计核心通过注入回调通知本节点发布 TF /odom /map_incremental
void LioNode::OnPose(const SE3& pose, double stamp) {
  // 发 TF(odom -> livox_frame) + /odom
  // TF 时间戳 = odom 消息时间戳 = 数据时间(参考 small_fast_lio publishOdometryAndTf)
  //   bag 回放时 launch 必须开 use_sim_time=true, 否则 rviz 按系统时间查 TF 会报
  //   "timestamp earlier than all data in transform cache"
  // 数据时间戳可能不在时钟域(设备 uptime), 统一加 time_offset_ 抬到时钟域
  CalibrateTimeOffset(stamp);
  const double stamp_out = stamp + time_offset_;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_out * 1e9));   // 时钟域时间
  tf.header.frame_id = common_params_.world_frame;   // odom(固定世界锚点)
  tf.child_frame_id = common_params_.body_frame;     // livox_frame(动)
  tf.transform.translation.x = pose.translation().x();
  tf.transform.translation.y = pose.translation().y();
  tf.transform.translation.z = pose.translation().z();
  Eigen::Quaterniond q(pose.rotationMatrix());
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(tf);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_out * 1e9));
  odom.header.frame_id = common_params_.world_frame;
  odom.child_frame_id = common_params_.body_frame;
  odom.pose.pose.position.x = pose.translation().x();
  odom.pose.pose.position.y = pose.translation().y();
  odom.pose.pose.position.z = pose.translation().z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  // twist: 速度取 ESKF 名义速度并转到 body 系(R_wb^T * v_w; ROS 约定 child_frame 系)
  const Eigen::Vector3d v_body = pose.rotationMatrix().transpose() * odometry_->GetVelocity();
  odom.twist.twist.linear.x = v_body.x();
  odom.twist.twist.linear.y = v_body.y();
  odom.twist.twist.linear.z = v_body.z();
  // 角速度未估计(ESKF 状态不含 omega; CT 连续时间运动隐含在 slerp 中), 保持 0
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom_pub_->publish(odom);

  if (path_pub_) {
    std::lock_guard<std::mutex> lk(path_mutex_);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose = odom.pose.pose;
    path_msg_.header = odom.header;
    path_msg_.poses.push_back(ps);
    path_pub_->publish(path_msg_);
  }
}

void LioNode::OnCloud(const std::vector<core::Point3D>& points, double stamp) {
  // 发布增量世界系点云: 当前帧新增点(frame = world_frame)
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(points.size());
  for (const auto& p : points) {
    pcl::PointXYZI pt;
    pt.x = p.point.x();
    pt.y = p.point.y();
    pt.z = p.point.z();
    pt.intensity = static_cast<float>(p.intensity);
    cloud.push_back(pt);
  }
  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  CalibrateTimeOffset(stamp);
  out.header.stamp = rclcpp::Time(static_cast<int64_t>((stamp + time_offset_) * 1e9));
  out.header.frame_id = common_params_.world_frame;
  map_pub_->publish(out);
}

// -------------------- 地图保存 --------------------
// 消费线程回调触发增量分片写盘; 退出时融合/体素滤波/清理分片
// 分片: chunk_<时间戳>.pcd(增量新点); 融合: map_fused_<时间戳>.pcd(全量)
void LioNode::OnMapChunk(const std::vector<core::Point3D>& points, double stamp) {
  if (points.empty()) return;
  // 增量分片 → pcl 点云 → 写盘(按帧尾时间戳命名)
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(points.size());
  for (const auto& p : points) {
    pcl::PointXYZI pt;
    pt.x = p.point.x();
    pt.y = p.point.y();
    pt.z = p.point.z();
    pt.intensity = static_cast<float>(p.intensity);
    cloud.push_back(pt);
  }
  // 时间戳命名: chunk_<秒>_<毫秒>.pcd(分片按时间序)
  int64_t sec = static_cast<int64_t>(stamp);
  int64_t ms = static_cast<int64_t>((stamp - static_cast<double>(sec)) * 1000.0);
  std::string path = save_map_params_.save_dir + "/chunk_" + std::to_string(sec) + "_" +
                     std::to_string(ms) + ".pcd";
  // 写盘失败(目录被删/磁盘满等)只告警不崩溃
  try {
    if (pcl::io::savePCDFileBinary(path, cloud) != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "failed to save chunk pcd: %s",
                           path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "map chunk saved: %s (%zu pts)", path.c_str(), cloud.size());
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "exception while saving chunk pcd (%s): %s", path.c_str(), e.what());
  }
}

void LioNode::FinalizeMapSave() {
  if (!save_map_params_.enabled) return;
  // 1) 扫描目录下所有分片 chunk_*.pcd
  namespace fs = std::filesystem;
  std::vector<std::string> chunk_files;
  if (!fs::exists(save_map_params_.save_dir)) {
    RCLCPP_WARN(get_logger(), "save dir not exist: %s", save_map_params_.save_dir.c_str());
    return;
  }
  for (const auto& entry : fs::directory_iterator(save_map_params_.save_dir)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("chunk_", 0) == 0 && name.size() > 4 &&
        name.compare(name.size() - 4, 4, ".pcd") == 0) {
      chunk_files.push_back(entry.path().string());
    }
  }
  std::sort(chunk_files.begin(), chunk_files.end());
  if (chunk_files.empty()) {
    RCLCPP_WARN(get_logger(), "no map chunks to fuse");
    return;
  }
  RCLCPP_INFO(get_logger(), "[map_save] fusing %zu chunks...", chunk_files.size());

  // 2) 逐个读取 → 拼接成全量
  pcl::PointCloud<pcl::PointXYZI> fused;
  for (const auto& f : chunk_files) {
    pcl::PointCloud<pcl::PointXYZI> chunk;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(f, chunk) == 0) {
      fused += chunk;
    } else {
      RCLCPP_WARN(get_logger(), "[map_save] failed to load chunk: %s", f.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "[map_save] fused %zu chunks -> %zu raw points", chunk_files.size(),
              fused.size());

  // 3) 体素滤波去重(分片重叠区会重复)
  pcl::PointCloud<pcl::PointXYZI> filtered;
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(fused.makeShared());
  vg.setLeafSize(save_map_params_.voxel_filter_size, save_map_params_.voxel_filter_size,
                 save_map_params_.voxel_filter_size);
  vg.filter(filtered);
  RCLCPP_INFO(get_logger(),
              "[map_save] voxel filter (leaf=%.2fm): %zu -> %zu points", save_map_params_.voxel_filter_size,
              fused.size(), filtered.size());

  // 4) 写融合结果(时间戳命名), 并清理分片; 写盘失败只告警不崩溃
  int64_t now_ms = static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  std::string fused_path = save_map_params_.save_dir + "/map_fused_" + std::to_string(now_ms) +
                           ".pcd";
  try {
    if (pcl::io::savePCDFileBinary(fused_path, filtered) == 0) {
      RCLCPP_INFO(get_logger(), "[map_save] fused map saved: %s (%zu pts)", fused_path.c_str(),
                  filtered.size());
      // 清理分片, 只留融合结果
      for (const auto& f : chunk_files) fs::remove(f);
      RCLCPP_INFO(get_logger(), "[map_save] cleaned %zu chunk files, keep: %s", chunk_files.size(),
                  fused_path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "[map_save] failed to save fused map: %s", fused_path.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "[map_save] exception while saving fused map (%s): %s",
                 fused_path.c_str(), e.what());
  }
}

}  // namespace ct_lio
