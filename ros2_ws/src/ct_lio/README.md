# CT-LIO ROS2 🚀

<img src="https://img.shields.io/badge/ROS2-Humble-blue" /> <img src="https://img.shields.io/badge/C%2B%2B-17-purple" /> <img src="https://img.shields.io/badge/Ceres-2.x-green" /> <img src="https://img.shields.io/badge/License-GPL--2.0-orange" />

> 一个轻量级的连续时间 LiDAR-Inertial 里程计 (CT-ICP + ESKF)，从 ROS1 逐行移植到 ROS2 🎯
> 核心算法与原版完全一致，只做工程化重构 ✨

本项目的算法移植自：

- [ct-lio](https://github.com/chengwei0427/ct-lio) — 连续时间 LiDAR-Inertial Odometry (CT-ICP + ESKF, 哈希体素地图) 📦
- [NV-LIO](https://github.com/hku-mars/NV-LIO) 思路参考 — LRU 体素地图淘汰 🌟
- [small_point_lio](https://github.com/Yancey2023/small_point_lio) 思路参考 — IMU 动态初始化模式 📖

## 做了什么优化？🤔

相比 ROS1 原版，核心算法零改动，主要做了以下性能/架构优化：

| 优化点 | 说明 | 复杂度对比 (ROS1 → ROS2) |
|--------|------|--------------------------|
| 入队零拷贝 🔄 | 回调按引用 + `std::move` 入队，原版按值拷贝整帧点云 | 每帧 O(N) 拷贝 → O(1) move |
| 回调轻量化 🧵 | 回调只做转换+入队，降采样移消费线程，不阻塞 spin | 回调内 O(N) 分桶 → 移出回调线程 |
| 删除冗余 shuffle | 原版回调内 shuffle×2 防扫描偏置，删除（降采样已移出） | 每帧 2×O(N) 洗牌 → 0 |
| LRU 地图淘汰 🗺️ | 体素地图 LRU 链表淘汰最久未用，替代原版每帧全图距离扫描 | 淘汰 O(V) 全扫/帧 → 插入时 O(1) 摊还 |
| 状态值语义 💾 | state 用值语义 + Clone()，原版裸指针 new/delete | 无泄漏，空间有界 |
| 帧尾 IMU 插值 🎯 | Predict 用线性插值合成帧尾时刻虚拟 IMU，原版直接用超帧 IMU 递推 | 初值更准，优化收敛更快 |

> 空间复杂度：地图 O(V)（V=体素数，LRU 容量上限约束），状态历史 O(帧数×状态大小)。

## 主要功能

- **IMU 初始化**：`init_mode: static`（要求静止，估计零偏+重力，收敛快）/ `dynamic`（攒够数据即可，只定重力方向，零偏在线学）。
- **重力对齐** 🧲：初始化时用 body 系实测重力方向构造初始姿态 `R0`（`FromTwoVectors`），world Z 严格竖到重力反方向 —— 即使设备初始安装倾斜，输出轨迹/地图也能自动摆正；重力大小取 `gravity_norm` 参数（默认 9.81，不同地点可改），不硬编码。
- **IMU 饱和检测**：剧烈运动超量程时按轴判定/外推，饱和帧冻结零偏标定（`check_satu` 等参数）。
- **时间戳域校准**：自动把设备 uptime 时间戳对齐到时钟域。
- **地图保存**：增量分片写 PCD，退出时融合 + 体素滤波 + 清理（保存目录自动创建，写盘失败只告警不崩溃）。

## 安装依赖

```bash
# ROS2 Humble 已装基础上
sudo apt install libceres-dev libpcl-dev libeigen3-dev
```

## 快速使用 🚗

```bash
cd ~/workSpace/lio_test_ws
colcon build --packages-select ct_lio_ros2
source install/setup.bash

# 终端 1: 回放 livox bag（bag 回放推荐 --clock，节点/TF 走模拟时钟）
ros2 bag play <your_livox_bag> --rate 1 --clock

# 终端 2: 启动节点（bag 回放配 use_sim_time:=true；真机实时默认 false）
ros2 launch ct_lio_ros2 ct_lio.launch.py use_sim_time:=true
```

## 输出话题 📡

| 话题(config 默认) | 类型 | 说明 |
|------|------|------|
| `/ct_lio/odom` | nav_msgs/Odometry | 位姿（含 ESKF 速度 twist；角速度为 0） |
| `/ct_lio/odometry_path` | nav_msgs/Path | 轨迹（`publish_path` 可关） |
| `/ct_lio/map_incremental` | PointCloud2 | 每帧新增世界系地图点 |
| `/tf` | TFMessage | `odom → <body_frame>`（config 可配，默认 livox_frame；数据时间戳） |

## License 📄

GPL-2.0-only（算法衍生自 [ct-lio](https://github.com/chengwei0427/ct-lio)，见 `package.xml`）。
