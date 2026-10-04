cmake_minimum_required(VERSION 3.8)
project(tree_mapper)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectDetection.msg"
  "msg/ObjectDetectionArray.msg"
  "msg/VegetationCandidate.msg"
  "msg/VegetationCandidateArray.msg"
  "srv/ExportSnapshot.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

install(PROGRAMS
  scripts/vegetation_candidate_detector.py
  scripts/vegetation_candidate_classifier.py
  scripts/object_id_tracker.py
  scripts/object_map_fuser.py
  scripts/tree_xy_plotter.py
  scripts/tree_cluster_xy_plotter.py
  scripts/tree_snapshot_exporter.py
  scripts/tree_map_csv_plotter.py
  scripts/tree_debug_monitor.py
  scripts/obstacle_cluster_node.py
  scripts/world_tree_ground_truth_plotter.py
  scripts/world_tree_compare_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/tree_mapper_runtime.py
  scripts/tree_mapper_ros2.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
