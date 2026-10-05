// params.hpp - 参数结构体(先声明默认值, 再由 config/launch 覆盖)
// 所有参数集中在 4 个结构体: CommonParams / PreprocessParams / OdometryParams / SaveMapParams
#pragma once

#include <string>
#include <vector>

namespace ct_lio {

/// 通用参数(话题名/frame名/QoS)
struct CommonParams {
  // ---- 话题(可配) ----
  std::string lidar_topic = "/livox/lidar";      // 点云输入(PointCloud2 only)
  std::string imu_topic = "/livox/imu";          // IMU 输入
  std::string odom_topic = "/odom";              // 位姿输出(可改名)
  std::string path_topic = "/odometry_path";     // 轨迹输出(可选)
  std::string scan_topic = "/map_incremental";   // 增量地图点云(可改名)

  // ---- 发布开关 ----
  bool publish_path = true;                      // 是否发布 /odometry_path

  // ---- 频率(订阅队列长度=freq*3) ----
  double lidar_freq_hz = 10.0;                   // 点云频率
  double imu_freq_hz = 200.0;                    // IMU 频率

  // ---- frame 名(只维护两个 frame) ----
  std::string world_frame = "map";        // world frame(世界系, 固定锚点; 原版语义=map/world)
  std::string body_frame = "livox_frame"; // 车体 frame(动)

  // ---- 时间戳域校准: 输出 TF/odom 对齐到时钟域 ----
  // 部分驱动把设备 uptime 秒当 header.stamp(如 mid360 开 PTP 无 master), 与时钟域差一个常数。
  // 0 = 自动校准(取首帧"时钟-数据"差值做偏移, bag 回放/真机通用); 非 0 = 手动指定偏移秒数。
  double time_offset = 0.0;
};

/// 预处理参数
struct PreprocessParams {
  int point_filter_num = 1;    // 跳点: 每 N 个留 1 个
  double blind = 0.1;          // 盲区半径(m)
  double max_range = 150.0;    // 点云有效距离(m): 超过丢弃(原版硬编码150)
};

/// 里程计参数(CT-ICP + ESKF)
struct OdometryParams {
  // ---- 外参: lidar -> IMU(统一来源) ----
  std::vector<double> extrinsic_t = {0.0, 0.0, 0.0};
  std::vector<double> extrinsic_r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  // ---- 同步 ----
  double delay_time = 0.15;    // 同步提前量(s)

  // ---- CT-ICP 匹配 ----
  double surf_res = 0.2;       // 面特征降采样体素(m)
  int max_num_iteration = 10;  // 外部迭代次数
  double downsample_size = 0.05;  // 消费线程入口粗降采样体素(m); 0=不降采样

  // ---- Ceres 求解器(参数化) ----
  int ceres_max_num_iterations = 5;   // 内层 LM 迭代上限
  int ceres_num_threads = 3;          // Ceres 线程数
  double ceres_huber_scale = 0.5;     // Huber 核宽度

  // ---- 体素地图 ----
  double size_voxel_map = 0.2;     // 地图体素边长(m)
  double min_distance_points = 0.05;  // 插入去重距离(m)
  int max_num_points_in_voxel = 20;   // 每体素最多点数
  int capacity = 1000000;             // LRU 容量(体素数)
  double max_distance = 500.0;        // 距离裁剪半径(m, 兜底, 主用 LRU)

  // ---- 邻域搜索 ----
  int voxel_neighborhood = 1;    // 邻域体素范围(1=3x3x3)
  int max_number_neighbors = 20; // 最多邻居数
  int threshold_voxel_occupancy = 1;
  int min_number_neighbors = 20;
  double power_planarity = 2.0;      // 平面性指数: a2D^power 作为残差权重(AddSurfCostFactors 用)
  int num_closest_neighbors = 1;     // 最近邻点数(AddSurfCostFactors 循环用)

  // ---- 残差 ----
  double max_dist_to_plane_icp = 0.3;
  double weight_alpha = 0.9;
  double weight_neighborhood = 0.1;
  int init_num_frames = 20;
  double sampling_rate = 1.0;
  int max_num_residuals = 2000;

  // ---- 一致性正则(CT 固定开启) ----
  double beta_location_consistency = 1.0;
  double beta_orientation_consistency = 1.0;
  double beta_small_velocity = 0.0;

  // ---- 收敛判定 ----
  double thres_translation_norm = 0.01;  // m
  double thres_orientation_norm = 0.1;   // deg

  // ---- ESKF ----
  double laser_point_cov = 0.001;
  double odom_trans_noise = 0.01;        // ObserveSE3 平移噪声
  double odom_ang_noise = 0.01;          // ObserveSE3 旋转噪声

  // ---- IMU 初始化 ----
  std::string init_mode = "static";      // static / dynamic
  double gravity_norm = 9.81;            // 重力大小(可配)
  double imu_dt = 0.01;                  // dt 上限判定阈值(5*imu_dt=0.05s 拒跳变; 非采样周期, livox 实为 200Hz=0.005s)
  double gyro_var = 1e-5;                // 陀螺测量标准差
  double acce_var = 1e-2;                // 加计测量标准差
  double bias_gyro_var = 1e-6;           // 陀螺零偏游走标准差
  double bias_acce_var = 1e-4;           // 加计零偏游走标准差
  double init_time_seconds = 1.0;        // static 静止收集时间
  int dynamic_min_imu_count = 200;       // dynamic: 攒够 IMU 数(原版硬编码200)
  double max_static_gyro_var = 0.5;      // 静止判定陀螺阈值
  double max_static_acce_var = 0.6;      // 静止判定加计阈值

  // ---- IMU 饱和检测(阈值判定方案) ----
  bool check_satu = true;                // 是否开启饱和检测(默认开)
  double satu_margin = 0.99;             // 饱和判定余量(×阈值, 防浮点误触发)
  double satu_gyro = 35.0;               // 陀螺饱和阈值(rad/s, ≈2005°/s, 按量程配)
  double satu_acc = 29.4;                // 加计饱和阈值(m/s², 3g×9.81, 按量程配)
  bool satu_hold = true;                 // 饱和时用上次有效值按轴外推(否则削顶值原样入队)
  bool satu_freeze_bias = true;          // 饱和帧跳过 IMU 初始化器(防零偏/重力估计污染); 稳态零偏由 LiDAR 观测更新(ESKF 设计)
};

/// 地图保存参数(增量分片 + 退出融合)
struct SaveMapParams {
  bool enabled = false;          // flag: 是否保存 pcd 文件
  int save_interval_frames = 100;  // 每 N 帧保存一个分片 pcd(增量新点)
  double voxel_filter_size = 0.2;  // 融合后体素滤波尺寸(m)
  std::string save_dir = "/tmp/ct_lio_map";  // 输出目录(分片/融合结果)
};

}  // namespace ct_lio
