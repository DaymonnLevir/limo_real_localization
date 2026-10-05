// lidar_odometry.hpp - LiDAR 里程计核心 (CT-ICP + ESKF 松耦合)
// 数据流: 数据入队 -> 同步打包 -> IMU初始化 -> Predict -> 初值 -> 建帧 -> 优化 -> 建图 -> ESKF观测
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "ct_lio_ros2/core/frame_types.hpp"
#include "ct_lio_ros2/core/ct_factors.hpp"
#include "ct_lio_ros2/core/rotation_parameterization.hpp"
#include "ct_lio_ros2/core/eskf.hpp"
#include "ct_lio_ros2/core/imu_initializer.hpp"
#include "ct_lio_ros2/core/voxel_map.hpp"
#include "ct_lio_ros2/core/nav_state.hpp"
#include "ct_lio_ros2/utils/eigen_types.hpp"
#include "ct_lio_ros2/utils/math_utils.hpp"
#include "ct_lio_ros2/utils/params.hpp"
#include "ct_lio_ros2/utils/timer.hpp"

namespace ct_lio::core {

/// 里程计核心(无 ROS 依赖)
class LidarOdometry {
 public:
  LidarOdometry() = default;
  ~LidarOdometry();

  /// 初始化: 参数 + 外参 + 构建组件(参数先全部到位再统一初始化)
  bool Initialize(const OdometryParams& params);

  // ---- 数据入队(生产线程调用) ----
  void PushLidarFrame(std::vector<Point3D> points, double begin_time, double timespan);
  void PushImu(const ImuPtr& imu);

  // ---- 消费线程主循环 ----
  void Run();
  // 优雅停止: 设置退出标志 + 唤醒等待中的消费线程(防析构死锁)
  void Stop() {
    running_ = false;
    cond_.notify_all();
  }

  // ---- 状态查询(供节点填 odom.twist) ----
  /// 当前名义速度(世界系, m/s; 来自 ESKF 状态 v; 未初始化时为零)
  Vec3d GetVelocity() const { return eskf_.GetNominalVel(); }

  // ---- 输出回调(由 node 层注入, 解耦 ROS) ----
  using PoseCallback = std::function<void(const SE3& pose, double stamp)>;
  using CloudCallback = std::function<void(const std::vector<Point3D>& points,
                                           double stamp)>;
  using MapChunkCallback = std::function<void(const std::vector<Point3D>& points,
                                              double stamp)>;
  void SetPoseCallback(PoseCallback cb) { pose_cb_ = std::move(cb); }
  void SetCloudCallback(CloudCallback cb) { cloud_cb_ = std::move(cb); }
  void SetMapChunkCallback(MapChunkCallback cb) { map_chunk_cb_ = std::move(cb); }

  // ---- 地图保存(增量分片 + 退出融合) ----
  /// 配置保存参数(enabled/间隔帧数); 未启用则零开销
  void SetMapSaveParams(const SaveMapParams& params) {
    save_params_ = params;
    save_frame_counter_ = 0;
  }
  /// 手动触发一次分片保存(供节点退出时收尾; 内部按间隔计数判断)
  void SaveMapChunk(double stamp);

 private:
  // ---- 流程步骤 ----
  void ProcessMeasurements(MeasurementGroup& m);
  void Predict(double lidar_end_time, const std::deque<ImuPtr>& imus);
  void SetInitialGuess();
  std::unique_ptr<CloudFrame> BuildFrame(std::vector<Point3D>& points);
  void EstimatePose(CloudFrame* frame);
  void Optimize(CloudFrame* frame);
  void UpdateMap(CloudFrame* frame);
  void CullMap();

  // ---- 辅助 ----
  std::vector<MeasurementGroup> GetMeasurements();
  void TransformPoint(Point3D& pt, const Eigen::Quaterniond& q_begin,
                      const Eigen::Quaterniond& q_end, const Eigen::Vector3d& t_begin,
                      const Eigen::Vector3d& t_end);
  void TransformKeypoints(std::vector<Point3D>& pts,
                          const Eigen::Quaterniond& q_begin, const Eigen::Quaterniond& q_end,
                          const Eigen::Vector3d& t_begin, const Eigen::Vector3d& t_end);
  std::vector<Eigen::Vector3d> FindNeighbors(const Eigen::Vector3d& point, int max_neighbors,
                                             int nb_voxels_visited = -1,
                                             int threshold_occupancy = -1);
  double CheckLocalizability(const std::vector<Eigen::Vector3d>& normals);
  void AddSurfCostFactors(std::vector<ceres::CostFunction*>& factors,
                          std::vector<Eigen::Vector3d>& normals,
                          std::vector<Point3D>& keypoints, CloudFrame* frame);
  static void VoxelDownsample(std::vector<Point3D>& in_out, double voxel_size);

  // ---- 参数与组件 ----
  OdometryParams params_;
  ESKFD eskf_;
  std::unique_ptr<ImuInitializer> imu_init_;
  VoxelMap voxel_map_;

  // ---- 外参: lidar -> imu (唯一来源; R/t 冗余副本已合并, 用 T_il_ 接口取) ----
  SE3 T_il_;                      // lidar -> imu
  double laser_sqrt_info_ = 0.0;  // 激光点协方差倒数开方(因子信息权重, 初始化时计算)

  // ---- 数据缓冲 ----
  std::deque<std::vector<Point3D>> lidar_buffer_;
  std::deque<std::pair<double, double>> time_buffer_;  // <begin_time, timespan>
  std::deque<ImuPtr> imu_buffer_;
  double last_lidar_time_ = -1.0;
  double last_imu_time_ = -1.0;
  double time_curr_ = 0.0;

  std::mutex mtx_buf_;
  std::condition_variable cond_;
  std::atomic<bool> running_{false};
  int frame_count_ = 0;   // 已处理帧计数(周期输出 Timer 用)

  // ---- 状态 ----
  State current_state_;
  std::vector<NavStated> imu_states_;        // Predict 期间的状态序列
  std::vector<State> state_history_;          // 历史 state(下一帧 begin 初值)
  ImuPtr last_imu_ = nullptr;
  int index_frame_ = 1;
  bool imu_need_init_ = true;

  // ---- 回调 ----
  PoseCallback pose_cb_;
  CloudCallback cloud_cb_;
  MapChunkCallback map_chunk_cb_;

  // ---- 地图保存(增量分片) ----
  SaveMapParams save_params_;              // 保存配置(默认关闭, 零开销)
  int save_frame_counter_ = 0;             // 距上次分片保存的帧计数
  std::vector<Point3D> pending_chunk_;  // 本分片内累积的增量新点(跨帧累积)

  // ---- 可视化 ----
  std::vector<Point3D> points_world_;  // 当前帧新增世界系点(发 /map_incremental)
};

}  // namespace ct_lio::core
