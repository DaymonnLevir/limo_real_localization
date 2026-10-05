cmake_minimum_required(VERSION 3.22)
project(ct_lio_ros2)

# 使用 find_package 的 <Package>_ROOT 变量策略(消除 CMP0074 dev 警告)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

# 第三方数值库 (系统级)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(fmt REQUIRED)

# 打包的第三方库 (sophus / tsl::robin_map)
# SYSTEM: 标记为系统头文件, 抑制内部 -Wpedantic 警告
# 弃用全局 include_directories, 改用 target 级(不污染其他 target/依赖)

# ================= 核心算法库 (与 ROS2 解耦, 仅依赖 sensor_msgs/pcl_conversions 消息类型, 便于测试) =================
add_library(ct_lio_core
  src/utils/timer.cpp
  src/sensor/point_cloud_preprocessor.cpp
  src/core/imu_initializer.cpp
  src/core/voxel_map.cpp
  src/core/ct_factors.cpp
  src/core/rotation_parameterization.cpp
  src/core/lidar_odometry.cpp
)
target_include_directories(ct_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# thirdparty 头文件被公开头(eigen_types.hpp→sophus, voxel_map.hpp→tsl)引用,
# 必须 PUBLIC 传播给链接方(node/test); SYSTEM 抑制 -Wpedantic
# BUILD_INTERFACE 保证安装后不把源码路径写进 INTERFACE(安装版见 install(DIRECTORY include/thirdparty))
target_include_directories(ct_lio_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/thirdparty>
)
target_link_libraries(ct_lio_core
  Ceres::ceres
  ${PCL_LIBRARIES}
  fmt::fmt
)
target_compile_definitions(ct_lio_core PUBLIC EIGEN_MPL2_ONLY)
# core 库使用 sensor_msgs 消息类型(PointCloud2)与 pcl_conversions
ament_target_dependencies(ct_lio_core
  sensor_msgs
  pcl_conversions
)

# ================= ROS2 节点 =================
add_executable(ct_lio_node
  src/ros/lio_node.cpp
  src/ros/main.cpp
)
target_include_directories(ct_lio_node PRIVATE
  include
)
target_link_libraries(ct_lio_node ct_lio_core)
ament_target_dependencies(ct_lio_node
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros pcl_conversions
)

# Protocol-focused smoke check: real Livox PointCloud2 layout and scan timing.
add_executable(ct_lio_timestamp_smoke test/timestamp_smoke.cpp)
target_link_libraries(ct_lio_timestamp_smoke ct_lio_core)
ament_target_dependencies(ct_lio_timestamp_smoke sensor_msgs)

# ================= 安装 =================
# 可执行文件
install(TARGETS ct_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

# 头文件 (H2: 补库安装接口)
# thirdparty(sophus/tsl)是头文件依赖, 必须随包安装, 否则下游 find_package 编译断链
#   安装到 include 根(sophus/ → include/sophus, tsl/ → include/tsl), 与 include 路径一致
install(DIRECTORY include/ct_lio_ros2
  DESTINATION include
)
install(DIRECTORY include/thirdparty/
  DESTINATION include
)

# 配置与 launch
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 导出库接口 (H2: 供其他包 find_package 复用 ct_lio_core)
install(TARGETS ct_lio_core
  EXPORT ct_lio_core_export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_include_directories(include)
ament_export_libraries(ct_lio_core)
ament_export_targets(ct_lio_core_export HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp sensor_msgs nav_msgs geometry_msgs
  tf2 tf2_ros pcl_conversions
  eigen3_cmake_module
  # 公开头文件依赖 Ceres/PCL, 必须导出, 下游 find_package 才能解析
  Ceres PCL
)

ament_package()
