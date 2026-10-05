// lidar_odometry.cpp - 里程计核心实现: CT-ICP 帧间优化 + ESKF 松耦合闭环
// 数据流: 入队 -> 同步打包 -> IMU初始化 -> Predict -> 初值 -> 建帧 -> 优化 -> 建图 -> ESKF观测
#include "ct_lio_ros2/core/lidar_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>

#include <ceres/ceres.h>

namespace ct_lio::core {


// 四元数角度距离(度)
static double AngularDistance(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
  double norm = ((a.toRotationMatrix() * b.toRotationMatrix().transpose()).trace() - 1.0) / 2.0;
  norm = std::acos(std::max(-1.0, std::min(1.0, norm))) * 180.0 / M_PI;
  return norm;
}

LidarOdometry::~LidarOdometry() {
  running_ = false;
  cond_.notify_all();
}

bool LidarOdometry::Initialize(const OdometryParams& params) {
  params_ = params;

  // ---- 参数校验(失败返回 false, 由节点层报错退出; 避免越界/非法配置崩溃) ----
  // 外参长度: vecFromArray 要求 3, matFromArray 要求 9(长度不符抛异常)
  if (params_.extrinsic_t.size() != 3 || params_.extrinsic_r.size() != 9) {
    std::cerr << "[LidarOdometry] Initialize failed: extrinsic_t.size()="
              << params_.extrinsic_t.size() << " (need 3), extrinsic_r.size()="
              << params_.extrinsic_r.size() << " (need 9)" << std::endl;
    return false;
  }
  if (params_.size_voxel_map <= 0.0 || params_.capacity <= 0 ||
      params_.max_num_points_in_voxel <= 0 || params_.min_distance_points < 0.0) {
    std::cerr << "[LidarOdometry] Initialize failed: map params must be positive "
                 "(size_voxel_map/capacity/max_num_points_in_voxel/min_distance_points)"
              << std::endl;
    return false;
  }

  // 外参(统一 T_il_ 单一来源; 长度已校验, 此处不应抛异常, try 兜底防御)
  Eigen::Vector3d t_il;
  Eigen::Matrix3d R_il;
  try {
    t_il = vecFromArray(params_.extrinsic_t);
    R_il = matFromArray(params_.extrinsic_r);
  } catch (const std::invalid_argument& e) {
    std::cerr << "[LidarOdometry] Initialize failed: " << e.what() << std::endl;
    return false;
  }
  Eigen::Quaterniond q_il(R_il);
  q_il.normalize();
  T_il_ = SE3(q_il, t_il);

  // 激光点协方差倒数开方(因子信息权重; 外参 T_il_ 由构造因子时传入)
  laser_sqrt_info_ = std::sqrt(1.0 / params_.laser_point_cov);

  // IMU 初始化器(static/dynamic 模式)
  ImuInitializer::Options imu_opt;
  imu_opt.gravity_norm = params_.gravity_norm;
  imu_opt.init_time_seconds = params_.init_time_seconds;
  imu_opt.max_static_gyro_var = params_.max_static_gyro_var;
  imu_opt.max_static_acce_var = params_.max_static_acce_var;
  imu_opt.dynamic_min_imu_count = params_.dynamic_min_imu_count;
  imu_init_ = std::make_unique<ImuInitializer>(imu_opt, params_.init_mode != "dynamic");
  // 模式由 params_.init_mode 决定
  // (ImuInitializer 内部 init_mode_static_ 默认 true; 这里通过参数控制)

  // 地图
  voxel_map_ = VoxelMap(params_.capacity, params_.size_voxel_map,
                             params_.max_num_points_in_voxel, params_.min_distance_points);

  return true;
}

// ---- 生产端: 数据入队 ----
// 生产端入队: 点云 + 时间戳入队, 唤醒消费线程(回调保持轻量)
void LidarOdometry::PushLidarFrame(std::vector<Point3D> points, double begin_time,
                                   double timespan) {
  // 锁内做回退清理, 避免与消费线程 GetMeasurements 的 front/pop 并发产生 UB
  {
    std::lock_guard<std::mutex> lk(mtx_buf_);
    if (begin_time < last_lidar_time_) {
      // bag 循环回退: 清队列 + 重置 time_curr_, 否则 imu.back - time_curr_ < delay_time 恒真 → 消费线程永久挂起
      lidar_buffer_.clear();
      time_buffer_.clear();
      time_curr_ = 0.0;
    }
    last_lidar_time_ = begin_time;
    lidar_buffer_.push_back(std::move(points));
    time_buffer_.emplace_back(begin_time, timespan);
  }
  cond_.notify_one();
}

// 生产端入队: IMU 指针入队, 唤醒消费线程
void LidarOdometry::PushImu(const ImuPtr& imu) {
  double t = imu->timestamp;
  // 锁内做回退清理: 此前 clear/last_imu_time_ 在锁外执行, 与消费线程 pop 并发为 UB
  {
    std::lock_guard<std::mutex> lk(mtx_buf_);
    if (t < last_imu_time_) {
      imu_buffer_.clear();
    }
    last_imu_time_ = t;
    imu_buffer_.push_back(imu);
  }
  cond_.notify_one();
}

// ---- 消费线程主循环 ----
// 消费线程主循环: 等待打包数据并逐个处理, 周期性输出耗时统计
void LidarOdometry::Run() {
  // CAS 原子交换防"Stop 后 Run 复活"竞态: 重复调用或竞态下仅一个线程进入循环
  bool expected = false;
  if (!running_.compare_exchange_strong(expected, true)) return;  // 已在运行, 忽略重复调用
  while (running_) {
    std::vector<MeasurementGroup> groups;
    {
      std::unique_lock<std::mutex> lk(mtx_buf_);
      cond_.wait(lk, [&] { return !running_ || !(groups = GetMeasurements()).empty(); });
      if (!running_ && groups.empty()) break;
    }

    for (auto& m : groups) {
      Timer::Evaluate([&] { ProcessMeasurements(m); }, "process_measurement");
    }

    // 周期性输出耗时统计并清空(默认每 100 帧; 防静默 + 防统计内存无界)
    if (++frame_count_ % 100 == 0) Timer::PrintAndReset();
  }
}

// ---- 同步打包: IMU 追上激光帧尾 + delay_time 才打包 ----
// 锁约定: 必须在持有 mtx_buf_ 锁时调用(生产/消费线程同锁保护)
//   返回后内部元素为 move 语义, 调用方不得复用缓冲
std::vector<MeasurementGroup> LidarOdometry::GetMeasurements() {
  std::vector<MeasurementGroup> measurements;
  while (true) {
    if (imu_buffer_.empty() || lidar_buffer_.empty()) return measurements;
    // 等 IMU 追上来
    if (imu_buffer_.back()->timestamp - time_curr_ < params_.delay_time) return measurements;

    MeasurementGroup m;
    m.lidar = std::move(lidar_buffer_.front());  // move 而非拷贝, 避免大向量复制
    const double lidar_begin_time = time_buffer_.front().first;
    m.lidar_end_time = lidar_begin_time + time_buffer_.front().second;
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    time_curr_ = m.lidar_end_time;

    double imu_time = imu_buffer_.front()->timestamp;
    while (!imu_buffer_.empty() && imu_time < m.lidar_end_time) {
      imu_time = imu_buffer_.front()->timestamp;
      if (imu_time > m.lidar_end_time) break;
      m.imu.push_back(imu_buffer_.front());
      imu_buffer_.pop_front();
    }
    if (!imu_buffer_.empty()) m.imu.push_back(imu_buffer_.front());  // 多塞一个供插值

    measurements.push_back(std::move(m));
  }
}

// ---- 帧处理主流程 ----
// 帧处理主流程: IMU初始化 -> 降采样 -> Predict -> 初值 -> 建帧 -> 优化建图 -> ESKF观测 -> 存档
void LidarOdometry::ProcessMeasurements(MeasurementGroup& m) {
  if (imu_need_init_) {
    // IMU 初始化: 用打包好的 IMU(全局 buffer 已被 pop 到 m.imu)
    for (auto& imu : m.imu) {
      // 零偏冻结: 饱和帧不参与初始化标定, 防饱和值污染零偏/重力估计
      if (params_.satu_freeze_bias && imu->saturated) continue;
      if (imu_init_->AddImu(*imu)) {
        ESKFD::Options opt;
        opt.imu_dt = params_.imu_dt;
        opt.gyro_var = params_.gyro_var;
        opt.acce_var = params_.acce_var;
        opt.bias_gyro_var = params_.bias_gyro_var;
        opt.bias_acce_var = params_.bias_acce_var;
        eskf_.SetInitialConditions(opt, imu_init_->GetInitBg(), imu_init_->GetInitBa(),
                                   imu_init_->GetGravity());
        std::cerr << "[LidarOdometry] IMU initialized, mode=" << params_.init_mode
                  << " bg=" << imu_init_->GetInitBg().transpose()
                  << " ba=" << imu_init_->GetInitBa().transpose() << std::endl;
        imu_need_init_ = false;
        break;
      }
    }
    return;
  }

  imu_states_.clear();

  // 粗降采样: 回调只入队, 降采样在消费线程匹配前做(原版回调内 subSampleFrame 0.01/0.05m 等价实现)
  if (params_.downsample_size > 0.0 && !m.lidar.empty()) {
    Timer::Evaluate([&] { VoxelDownsample(m.lidar, params_.downsample_size); },
                           "coarse_downsample");
  }

  // IMU 前向传播
  Timer::Evaluate([&] { Predict(m.lidar_end_time, m.imu); }, "predict");
  // 位姿初值
  Timer::Evaluate([&] { SetInitialGuess(); }, "state_init");

  // 建帧(点转世界系 + alpha)
  std::unique_ptr<CloudFrame> frame;
  Timer::Evaluate([&] { frame = BuildFrame(m.lidar); }, "build_frame");

  // 位姿估计(优化 + 建图 + 裁剪)
  Timer::Evaluate([&] { EstimatePose(frame.get()); }, "estimate_pose");

  // 发布在优化后、ESKF 观测前(用优化后位姿做观测初值)
  SE3 pose_of_lo(current_state_.rotation, current_state_.translation);
  if (pose_cb_) pose_cb_(pose_of_lo, m.lidar_end_time);
  // points_world_ 无论回调是否存在都清空, 防止核心单独使用(无回调注入)时无限累积
  if (!points_world_.empty()) {
    if (cloud_cb_) cloud_cb_(points_world_, m.lidar_end_time);
    points_world_.clear();
  }

  // ---- 地图保存(增量分片): 启用时把本帧新增点累积进当前分片, 按间隔触发 ----
  if (save_params_.enabled) {
    // points_world_ 已 clear, 从 UpdateMap 的累积缓冲取本帧增量(见 UpdateMap)
    // 这里只需推进计数并按间隔触发; 点累积在 UpdateMap 里做
    save_frame_counter_++;
    if (save_frame_counter_ >= save_params_.save_interval_frames) {
      SaveMapChunk(m.lidar_end_time);
    }
  }

  // ESKF 观测(松耦合闭环)
  Timer::Evaluate(
      [&] { eskf_.ObserveSE3(pose_of_lo, params_.odom_trans_noise, params_.odom_ang_noise); },
      "eskf_obs");

  // 状态存档 + 滚动
  state_history_.push_back(current_state_.Clone());
  current_state_.RollToNextFrame();

  index_frame_++;
}

// IMU 前向传播: 帧内 IMU 直接递推, 帧尾用"多塞"IMU 线性插值合成虚拟 IMU
void LidarOdometry::Predict(double lidar_end_time, const std::deque<ImuPtr>& imus) {
  imu_states_.emplace_back(eskf_.GetNominalState());  // 起点状态(上一帧末尾)

  for (auto& imu : imus) {
    if (imu->timestamp <= lidar_end_time) {
      // 帧内 IMU: 直接递推
      if (last_imu_ == nullptr) last_imu_ = imu;
      eskf_.Predict(*imu);
      imu_states_.emplace_back(eskf_.GetNominalState());
      last_imu_ = imu;
    } else {
      // 超过帧尾的那个"多塞"IMU: 线性插值合成帧尾时刻的虚拟 IMU(参考可跑通版)
      // 判空保护: 若本帧无帧内 IMU(last_imu_ 仍为 nullptr), 直接用该 IMU 递推
      if (last_imu_ == nullptr) {
        eskf_.Predict(*imu);
        imu_states_.emplace_back(eskf_.GetNominalState());
        last_imu_ = imu;
        continue;
      }
      double dt_1 = imu->timestamp - lidar_end_time;
      double dt_2 = lidar_end_time - last_imu_->timestamp;
      if (dt_1 + dt_2 < 1e-6) break;
      double w1 = dt_1 / (dt_1 + dt_2);
      double w2 = dt_2 / (dt_1 + dt_2);
      Eigen::Vector3d acc_temp = w1 * last_imu_->acce + w2 * imu->acce;
      Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro + w2 * imu->gyro;
      auto imu_temp = std::make_shared<ImuMeasurement>(lidar_end_time, gyr_temp, acc_temp);
      // 透传饱和标记: 多塞帧若饱和, 插值结果也应标记
      imu_temp->saturated = imu->saturated;
      eskf_.Predict(*imu_temp);
      imu_states_.emplace_back(eskf_.GetNominalState());
      last_imu_ = imu_temp;
    }
  }
}

// 位姿初值: 首帧用 IMU 状态序列首尾, 后续帧 begin 取上一帧 end 状态
void LidarOdometry::SetInitialGuess() {
  if (index_frame_ < 2) {
    // 第 1 帧: 用 IMU 状态序列首尾; 若空则保持默认(原点, odom 锚点)
    if (!imu_states_.empty()) {
      current_state_.rotation_begin = imu_states_.front().R.unit_quaternion();
      current_state_.translation_begin = imu_states_.front().p;
      current_state_.rotation = imu_states_.back().R.unit_quaternion();
      current_state_.translation = imu_states_.back().p;
    }
  } else {
    // 后续帧: begin = 上一帧 end(历史), end = IMU 预测
    if (!state_history_.empty()) {
      current_state_.rotation_begin = state_history_.back().rotation;
      current_state_.translation_begin = state_history_.back().translation;
    }
    if (!imu_states_.empty()) {
      current_state_.rotation = imu_states_.back().R.unit_quaternion();
      current_state_.translation = imu_states_.back().p;
    }
  }
}

// 建帧: 打包点云与状态, 逐点按 alpha 插值转世界系(隐式去畸变)
std::unique_ptr<CloudFrame> LidarOdometry::BuildFrame(std::vector<Point3D>& points) {
  auto frame = std::make_unique<CloudFrame>();
  frame->point_surf = points;
  frame->frame_id = index_frame_;
  frame->state = current_state_;

  if (index_frame_ < 2) {
    for (auto& p : frame->point_surf) p.alpha_time = 1.0;
  }

  // 逐点转世界系(CT 模式: 按 alpha 插值)
  for (auto& p : frame->point_surf) {
    TransformPoint(p, current_state_.rotation_begin, current_state_.rotation,
                   current_state_.translation_begin, current_state_.translation);
  }
  return frame;
}

// 单点 CT 变换: 按 alpha 插值位姿(隐式去畸变)
void LidarOdometry::TransformPoint(Point3D& pt, const Eigen::Quaterniond& q_begin,
                                   const Eigen::Quaterniond& q_end,
                                   const Eigen::Vector3d& t_begin, const Eigen::Vector3d& t_end) {
  // CT 模式: 每个点按 alpha 插值位姿(隐式去畸变)
  Eigen::Quaterniond q = q_begin.slerp(pt.alpha_time, q_end);
  q.normalize();
  Eigen::Vector3d t = t_begin * (1.0 - pt.alpha_time) + t_end * pt.alpha_time;
  pt.point = q * (T_il_.rotationMatrix() * pt.raw_point + T_il_.translation()) + t;
}

// 批量点 CT 变换
void LidarOdometry::TransformKeypoints(std::vector<Point3D>& pts,
                                       const Eigen::Quaterniond& q_begin,
                                       const Eigen::Quaterniond& q_end,
                                       const Eigen::Vector3d& t_begin,
                                       const Eigen::Vector3d& t_end) {
  for (auto& p : pts) TransformPoint(p, q_begin, q_end, t_begin, t_end);
}

// ---- 位姿估计/优化 ----
// 位姿估计: 优化 + 建图 + 裁剪
void LidarOdometry::EstimatePose(CloudFrame* frame) {
  if (index_frame_ > 1) {
    Timer::Evaluate([&] { Optimize(frame); }, "optimize");
  }
  Timer::Evaluate([&] { UpdateMap(frame); }, "map_update");
  Timer::Evaluate([&] { CullMap(); }, "fov_segment");
}

// ---- optimize: CT-ICP 匹配核心 ----
// CT-ICP 优化: 多轮 Ceres 迭代(点面残差 + 一致性正则), 收敛判定后写回状态
void LidarOdometry::Optimize(CloudFrame* frame) {
  State& s = frame->state;
  Eigen::Quaterniond begin_quat = s.rotation_begin;
  Eigen::Quaterniond end_quat = s.rotation;
  Eigen::Vector3d begin_t = s.translation_begin;
  Eigen::Vector3d end_t = s.translation;

  // 上一帧状态(正则项)
  Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();
  if (!state_history_.empty()) {
    previous_translation = state_history_.back().translation;
    previous_orientation = state_history_.back().rotation;
  }

  // 降采样关键点
  std::vector<Point3D> surf_keypoints = frame->point_surf;
  VoxelDownsample(surf_keypoints, params_.surf_res * params_.sampling_rate);

  auto transform_keypoints = [&](std::vector<Point3D>& pts) {
    TransformKeypoints(pts, begin_quat, end_quat, begin_t, end_t);
  };

  for (int iter = 0; iter < params_.max_num_iteration; ++iter) {
    transform_keypoints(surf_keypoints);

    ceres::LossFunction* loss = new ceres::HuberLoss(params_.ceres_huber_scale);
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    // 每个参数块独立 LocalParameterization 实例, 避免所有权依赖 Ceres 版本
    problem.AddParameterBlock(begin_quat.coeffs().data(), 4, new RotationParameterization());
    problem.AddParameterBlock(end_quat.coeffs().data(), 4, new RotationParameterization());
    problem.AddParameterBlock(begin_t.data(), 3);
    problem.AddParameterBlock(end_t.data(), 3);

    std::vector<ceres::CostFunction*> surf_factors;
    std::vector<Eigen::Vector3d> normals;
    AddSurfCostFactors(surf_factors, normals, surf_keypoints, frame);

    // 退化检测(原版 checkLocalizability): 法向量 SVD 最小奇异值 < 3.5 提示退化场景(长廊/平面)
    // 仅警告, 不影响优化(原版同款)
    CheckLocalizability(normals);

    int surf_num = 0;
    for (auto* f : surf_factors) {
      surf_num++;
      problem.AddResidualBlock(f, loss, begin_t.data(), begin_quat.coeffs().data(), end_t.data(),
                               end_quat.coeffs().data());
    }

    // 一致性正则
    if (params_.beta_location_consistency > 0.0) {
      auto* cost = new LocationConsistencyFactor(
          previous_translation, std::sqrt(surf_num * params_.beta_location_consistency *
                                          params_.laser_point_cov));
      problem.AddResidualBlock(cost, nullptr, begin_t.data());
    }
    if (params_.beta_orientation_consistency > 0.0) {
      auto* cost = new RotationConsistencyFactor(
          previous_orientation, std::sqrt(surf_num * params_.beta_orientation_consistency *
                                          params_.laser_point_cov));
      problem.AddResidualBlock(cost, nullptr, begin_quat.coeffs().data());
    }
    if (params_.beta_small_velocity > 0.0) {
      auto* cost = new SmallVelocityFactor(
          std::sqrt(surf_num * params_.beta_small_velocity * params_.laser_point_cov));
      problem.AddResidualBlock(cost, nullptr, begin_t.data(), end_t.data());
    }

    ceres::Solver::Options options;
    options.max_num_iterations = params_.ceres_max_num_iterations;
    options.num_threads = params_.ceres_num_threads;
    options.minimizer_progress_to_stdout = false;
    options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
      // 求解失败降级为日志 + 跳帧, 防异常穿透消费线程导致整进程崩溃
      std::cerr << "[LidarOdometry] optimize failed, skip this iteration: "
                << summary.message << std::endl;
      break;
    }

    begin_quat.normalize();
    end_quat.normalize();

    // 收敛判定: 对比 current_state_(上一轮写回后的值) 与本轮结果, 先算 diff 再写回
    double diff_trans = (current_state_.translation_begin - begin_t).norm() +
                        (current_state_.translation - end_t).norm();
    double diff_rot = AngularDistance(current_state_.rotation_begin, begin_quat) +
                      AngularDistance(current_state_.rotation, end_quat);

    // 每轮写回: 写回后下一轮 diff 基准才是"上一轮结果"
    s.rotation_begin = begin_quat;
    s.translation_begin = begin_t;
    s.rotation = end_quat;
    s.translation = end_t;
    current_state_.rotation_begin = begin_quat;
    current_state_.translation_begin = begin_t;
    current_state_.rotation = end_quat;
    current_state_.translation = end_t;

    if (diff_rot < params_.thres_orientation_norm &&
        diff_trans < params_.thres_translation_norm) {
      break;
    }
  }

  // 用最终 begin/end 重转全部点(供插地图)
  transform_keypoints(frame->point_surf);
}

// ---- 残差构造/邻域搜索 ----
// 残差构造: 邻居搜索 + PCA 法向量 + 平面性权重, 生成点面因子(外点剔除 + 残差上限)
void LidarOdometry::AddSurfCostFactors(std::vector<ceres::CostFunction*>& factors,
                                       std::vector<Eigen::Vector3d>& normals,
                                       std::vector<Point3D>& keypoints, CloudFrame* frame) {
  double lambda_weight = std::abs(params_.weight_alpha);
  double lambda_neighborhood = std::abs(params_.weight_neighborhood);
  const double sum = lambda_weight + lambda_neighborhood;
  lambda_weight /= sum;
  lambda_neighborhood /= sum;

  const double kMaxPointToPlane = params_.max_dist_to_plane_icp;
  // 前 init_num_frames 帧(初始化期)邻域加宽, 增强早期匹配鲁棒性(原版 ct-lio 行为)
  const int nb_voxels =
      frame->frame_id < params_.init_num_frames ? 2 : params_.voxel_neighborhood;
  const int kThreshold =
      frame->frame_id < params_.init_num_frames ? 1 : params_.threshold_voxel_occupancy;

  for (auto& keypoint : keypoints) {
    // 邻居搜索(3x3x3 邻域 + 最近 max_number_neighbors; 前 init_num_frames 帧加宽到 2)
    auto neighbors =
        FindNeighbors(keypoint.point, params_.max_number_neighbors, nb_voxels, kThreshold);
    if (static_cast<int>(neighbors.size()) < params_.min_number_neighbors) continue;

    // PCA 拟合平面
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& n : neighbors) center += n;
    center /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& n : neighbors) {
      Eigen::Vector3d diff = n - center;
      cov += diff * diff.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d normal = es.eigenvectors().col(0).normalized();

    // 法向量朝向: 指向传感器
    if (normal.dot(frame->state.translation_begin - keypoint.point) < 0) {
      normal = -normal;
    }

    // 平面性权重
    double sigma1 = std::sqrt(std::abs(es.eigenvalues()[2]));
    double sigma2 = std::sqrt(std::abs(es.eigenvalues()[1]));
    double sigma3 = std::sqrt(std::abs(es.eigenvalues()[0]));
    double a2d = (sigma2 - sigma3) / (sigma1 + 1e-9);
    double planarity = std::pow(a2d, params_.power_planarity);

    double weight = lambda_weight * planarity +
                    lambda_neighborhood *
                        std::exp(-(neighbors[0] - keypoint.point).norm() /
                                 (kMaxPointToPlane * params_.min_number_neighbors));

    // 点到面硬门槛(0.3m)外点剔除 + 最近邻 offset + 残差上限
    for (int i = 0; i < params_.num_closest_neighbors; ++i) {
      double point_to_plane_dist = std::abs((keypoint.point - neighbors[i]).transpose() * normal);
      if (point_to_plane_dist >= kMaxPointToPlane) continue;  // 外点剔除: 点到面距离超阈值

      Eigen::Vector3d norm_vector = normal;
      norm_vector.normalize();
      normals.push_back(norm_vector);

      // 平面 offset 用最近邻点(非质心)
      double norm_offset = -norm_vector.dot(neighbors[i]);

      auto* factor = new CtPointToPlaneFactor(keypoint.raw_point, norm_vector, norm_offset,
                                              keypoint.alpha_time, T_il_.translation(),
                                              Eigen::Quaterniond(T_il_.rotationMatrix()),
                                              laser_sqrt_info_, weight);
      factors.push_back(factor);

      if (static_cast<int>(factors.size()) >= params_.max_num_residuals) break;  // 残差上限
    }
    if (static_cast<int>(factors.size()) >= params_.max_num_residuals) break;
  }
}

// ---- 邻居搜索(3x3x3 体素邻域 + 优先队列 Top-N) ----
// 邻居搜索: 3x3x3 体素邻域内取最近 Top-N(优先队列维护, 支持邻域加宽与占用阈值)
std::vector<Eigen::Vector3d> LidarOdometry::FindNeighbors(const Eigen::Vector3d& point,
                                                          int max_neighbors,
                                                          int nb_voxels_visited,
                                                          int threshold_occupancy) {
  using QItem = std::tuple<double, Eigen::Vector3d>;
  auto cmp = [](const QItem& a, const QItem& b) { return std::get<0>(a) < std::get<0>(b); };
  std::priority_queue<QItem, std::vector<QItem>, decltype(cmp)> pq(cmp);

  short kx = static_cast<short>(point.x() / params_.size_voxel_map);
  short ky = static_cast<short>(point.y() / params_.size_voxel_map);
  short kz = static_cast<short>(point.z() / params_.size_voxel_map);

  const int nb = (nb_voxels_visited >= 0) ? nb_voxels_visited : params_.voxel_neighborhood;
  Voxel v(kx, ky, kz);
  for (short kxx = kx - nb; kxx <= kx + nb; ++kxx) {
    for (short kyy = ky - nb; kyy <= ky + nb; ++kyy) {
      for (short kzz = kz - nb; kzz <= kz + nb; ++kzz) {
        v.x = kxx;
        v.y = kyy;
        v.z = kzz;
        auto block = voxel_map_.GetBlock(v);
        if (!block) continue;
        if (block->NumPoints() < threshold_occupancy) continue;
        for (const auto& neighbor : block->Points()) {
          double dist = (neighbor - point).norm();
          if (static_cast<int>(pq.size()) == max_neighbors) {
            if (dist < std::get<0>(pq.top())) {
              pq.pop();
              pq.emplace(dist, neighbor);
            }
          } else {
            pq.emplace(dist, neighbor);
          }
        }
      }
    }
  }

  std::vector<Eigen::Vector3d> out(pq.size());
  for (int i = static_cast<int>(pq.size()) - 1; i >= 0; --i) {
    out[i] = std::get<1>(pq.top());
    pq.pop();
  }
  return out;
}

// ---- 建图与裁剪 ----
// 建图: 插入体素地图, 并收集本帧新增点供可视化/分片保存
void LidarOdometry::UpdateMap(CloudFrame* frame) {
  for (auto& p : frame->point_surf) {
    voxel_map_.InsertPoint(p.point);
  }
  // 收集本帧新增点(发可视化)
  points_world_.insert(points_world_.end(), frame->point_surf.begin(), frame->point_surf.end());
  // 地图保存启用时: 同批点累积进当前分片(增量语义, 与可视化一致)
  if (save_params_.enabled) {
    pending_chunk_.insert(pending_chunk_.end(), frame->point_surf.begin(),
                          frame->point_surf.end());
  }
}

/// 触发一次分片保存: 把累积的增量新点交给回调并清空(按间隔或退出收尾)
void LidarOdometry::SaveMapChunk(double stamp) {
  if (!save_params_.enabled || pending_chunk_.empty()) return;
  if (map_chunk_cb_) map_chunk_cb_(pending_chunk_, stamp);
  pending_chunk_.clear();
  save_frame_counter_ = 0;
}

// 裁剪: LRU 已自动淘汰, 此处额外做距离兜底(max_distance 外体素剔除)
void LidarOdometry::CullMap() {
  // LRU 自动淘汰之外, 额外做距离兜底裁剪(max_distance)
  if (params_.max_distance <= 0.0) return;
  Eigen::Vector3d loc = current_state_.translation;
  double max_sq = params_.max_distance * params_.max_distance;
  std::vector<Voxel> to_erase;
  voxel_map_.ForEachBlock([&](const Voxel& v, const VoxelBlock& block) {
    if (block.NumPoints() > 0) {
      double sq = (block.Points()[0] - loc).squaredNorm();
      if (sq > max_sq) to_erase.push_back(v);
    }
  });
  for (const auto& v : to_erase) voxel_map_.EraseVoxel(v);
}

// ---- 工具 ----
// 体素降采样: 按 voxel 坐标保留每格首点
void LidarOdometry::VoxelDownsample(std::vector<Point3D>& in_out, double voxel_size) {
  if (voxel_size <= 0.0) return;
  std::vector<Point3D> out;
  out.reserve(in_out.size());
  // 简单体素: 按 voxel 坐标保留每格第一个
  tsl::robin_map<Voxel, Point3D> grid;
  for (auto& p : in_out) {
    Voxel v = Voxel::FromPoint(p.point, voxel_size);
    if (grid.find(v) == grid.end()) {
      grid.insert({v, p});
    }
  }
  for (auto& [v, p] : grid) {
    (void)v;
    out.push_back(p);
  }
  in_out.swap(out);
}

// 退化检测: 法向量 SVD 最小奇异值 < 3.5 提示退化场景(长廊/平面), 仅警告不影响优化
double LidarOdometry::CheckLocalizability(const std::vector<Eigen::Vector3d>& normals) {
  // 退化判定阈值: 法向量 SVD 最小奇异值(原版 ct-lio 硬编码 3.5, 命名化便于维护)
  constexpr double kMinSingularValue = 3.5;
  if (normals.size() < 10) return -1.0;
  Eigen::MatrixXd mat(normals.size(), 3);
  for (size_t i = 0; i < normals.size(); ++i) mat.row(i) = normals[i].transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double smallest = svd.singularValues()(2);
  if (smallest < kMinSingularValue) {
    // 原版 ct-lio: 最小奇异值 < 3.5 提示退化场景(长廊/平面), 仅警告
    std::cerr << "[LidarOdometry] Low localizability: singular values = "
              << svd.singularValues()(0) << ", " << svd.singularValues()(1) << ", "
              << svd.singularValues()(2) << std::endl;
  }
  return smallest;
}

}  // namespace ct_lio::core
