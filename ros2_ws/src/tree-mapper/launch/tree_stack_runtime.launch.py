import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_RUN_ID = time.strftime("run_%Y%m%d_%H%M%S", time.localtime())


def generate_launch_description():
    pkg_share = get_package_share_directory("tree_mapper")
    default_config = os.path.join(pkg_share, "config", "interface_topics.yaml")

    args = [
        DeclareLaunchArgument("config_yaml", default_value=default_config),
        DeclareLaunchArgument("run_plotter", default_value="true"),
        DeclareLaunchArgument("run_cluster_plotter", default_value="true"),
        DeclareLaunchArgument("run_fuser", default_value="true"),
        DeclareLaunchArgument("run_snapshot_exporter", default_value="true"),
        DeclareLaunchArgument("candidate_history_csv_output_path", default_value="auto"),
        DeclareLaunchArgument("tree_fuser_csv_output_path", default_value="auto"),
        DeclareLaunchArgument("tree_fuser_json_output_path", default_value="auto"),
        DeclareLaunchArgument("tree_fuser_history_csv_output_path", default_value="auto"),
        DeclareLaunchArgument("shrub_fuser_csv_output_path", default_value="auto"),
        DeclareLaunchArgument("shrub_fuser_json_output_path", default_value="auto"),
        DeclareLaunchArgument("shrub_fuser_history_csv_output_path", default_value="auto"),
        DeclareLaunchArgument("run_output_dir", default_value=""),
        DeclareLaunchArgument("run_id", default_value=DEFAULT_RUN_ID),
        DeclareLaunchArgument("runtime_state_namespace", default_value="/tree_mapper_runtime"),
        DeclareLaunchArgument("fixed_axes", default_value="false"),
        DeclareLaunchArgument("x_min", default_value="-10.0"),
        DeclareLaunchArgument("x_max", default_value="10.0"),
        DeclareLaunchArgument("y_min", default_value="-8.0"),
        DeclareLaunchArgument("y_max", default_value="7.0"),
        DeclareLaunchArgument("axis_margin", default_value="5.0"),
        DeclareLaunchArgument("use_boundary_bounds", default_value="false"),
        DeclareLaunchArgument("tick_step", default_value="5.0"),
    ]

    config_yaml = LaunchConfiguration("config_yaml")

    actions = [
        Node(
            package="tree_mapper",
            executable="vegetation_candidate_detector.py",
            name="vegetation_candidate_detector",
            output="screen",
            parameters=[
                config_yaml,
                {
                    "output_array_topic": "/vegetation_mapper/internal/candidates_full",
                    "cluster_points_topic": "/vegetation_mapper/internal/cluster_points",
                    "cluster_labels_topic": "/vegetation_mapper/internal/cluster_labels",
                    "history_csv_output_path": LaunchConfiguration("candidate_history_csv_output_path"),
                    "run_output_dir": LaunchConfiguration("run_output_dir"),
                    "run_id": LaunchConfiguration("run_id"),
                    "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="vegetation_candidate_classifier.py",
            name="vegetation_candidate_classifier",
            output="screen",
            parameters=[
                config_yaml,
                {
                    "input_array_topic": "/vegetation_mapper/internal/candidates_full",
                    "tree_output_topic": "/tree_mapper/internal/detections_legacy",
                    "tree_output_array_topic": "/tree_mapper/internal/detections_full",
                    "tree_marker_topic": "/tree_mapper/internal/detections_raw_markers",
                    "tree_radius_topic": "/tree_mapper/internal/detection_radii",
                    "shrub_output_topic": "/shrub_mapper/internal/detections_legacy",
                    "shrub_output_array_topic": "/shrub_mapper/internal/detections_full",
                    "shrub_marker_topic": "/shrub_mapper/internal/detections_raw_markers",
                    "shrub_radius_topic": "/shrub_mapper/internal/detection_radii",
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="object_id_tracker.py",
            name="tree_id_tracker",
            output="screen",
            parameters=[
                config_yaml,
                {
                    "class_label": "tree",
                    "input_array_topic": "/tree_mapper/internal/detections_full",
                    "output_topic": "/tree_mapper/internal/tracked_legacy",
                    "output_array_topic": "/tree_mapper/internal/tracked_full",
                    "id_topic": "/tree_mapper/internal/tracked_ids",
                    "marker_topic": "/tree_mapper/internal/tracked_markers",
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="object_id_tracker.py",
            name="shrub_id_tracker",
            output="screen",
            parameters=[
                config_yaml,
                {
                    "class_label": "shrub",
                    "input_array_topic": "/shrub_mapper/internal/detections_full",
                    "output_topic": "/shrub_mapper/internal/tracked_legacy",
                    "output_array_topic": "/shrub_mapper/internal/tracked_full",
                    "id_topic": "/shrub_mapper/internal/tracked_ids",
                    "marker_topic": "/shrub_mapper/internal/tracked_markers",
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="object_map_fuser.py",
            name="tree_map_fuser",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_fuser")),
            parameters=[
                config_yaml,
                {
                    "class_label": "tree",
                    "array_topic": "/tree_mapper/internal/tracked_full",
                    "csv_output_path": LaunchConfiguration("tree_fuser_csv_output_path"),
                    "json_output_path": LaunchConfiguration("tree_fuser_json_output_path"),
                    "history_csv_output_path": LaunchConfiguration("tree_fuser_history_csv_output_path"),
                    "run_output_dir": LaunchConfiguration("run_output_dir"),
                    "run_id": LaunchConfiguration("run_id"),
                    "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="object_map_fuser.py",
            name="shrub_map_fuser",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_fuser")),
            parameters=[
                config_yaml,
                {
                    "class_label": "shrub",
                    "array_topic": "/shrub_mapper/internal/tracked_full",
                    "output_topic": "/shrub_map_poses",
                    "output_id_topic": "/shrub_map_ids",
                    "output_array_topic": "/shrub_map_full",
                    "candidate_topic": "/shrub_map_candidates",
                    "candidate_id_topic": "/shrub_map_candidate_ids",
                    "candidate_array_topic": "/shrub_map_candidate_full",
                    "marker_topic": "/shrub_map_markers",
                    "csv_output_path": LaunchConfiguration("shrub_fuser_csv_output_path"),
                    "json_output_path": LaunchConfiguration("shrub_fuser_json_output_path"),
                    "history_csv_output_path": LaunchConfiguration("shrub_fuser_history_csv_output_path"),
                    "run_output_dir": LaunchConfiguration("run_output_dir"),
                    "run_id": LaunchConfiguration("run_id"),
                    "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="tree_snapshot_exporter.py",
            name="tree_snapshot_exporter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_snapshot_exporter")),
            parameters=[
                config_yaml,
                {
                    "cluster_points_topic": "/vegetation_mapper/internal/cluster_points",
                    "cluster_labels_topic": "/vegetation_mapper/internal/cluster_labels",
                    "map_array_topic": "/tree_map_full",
                    "map_candidate_array_topic": "/tree_map_candidate_full",
                    "fixed_axes": LaunchConfiguration("fixed_axes"),
                    "x_min": LaunchConfiguration("x_min"),
                    "x_max": LaunchConfiguration("x_max"),
                    "y_min": LaunchConfiguration("y_min"),
                    "y_max": LaunchConfiguration("y_max"),
                    "axis_margin": LaunchConfiguration("axis_margin"),
                    "use_boundary_bounds": LaunchConfiguration("use_boundary_bounds"),
                    "tick_step": LaunchConfiguration("tick_step"),
                    "run_output_dir": LaunchConfiguration("run_output_dir"),
                    "run_id": LaunchConfiguration("run_id"),
                    "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
                    "tree_map_csv_output_path": LaunchConfiguration("tree_fuser_csv_output_path"),
                    "tree_map_json_output_path": LaunchConfiguration("tree_fuser_json_output_path"),
                    "tree_map_history_output_path": LaunchConfiguration("tree_fuser_history_csv_output_path"),
                    "tree_detection_history_output_path": LaunchConfiguration("candidate_history_csv_output_path"),
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="tree_xy_plotter.py",
            name="tree_xy_plotter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_plotter")),
            parameters=[
                config_yaml,
                {
                    "live_array_topic": "/tree_mapper/internal/tracked_full",
                    "map_array_topic": "/tree_map_full",
                    "map_candidate_array_topic": "/tree_map_candidate_full",
                    "shrub_live_array_topic": "/shrub_mapper/internal/tracked_full",
                    "shrub_map_array_topic": "/shrub_map_full",
                    "shrub_map_candidate_array_topic": "/shrub_map_candidate_full",
                    "fixed_axes": LaunchConfiguration("fixed_axes"),
                    "x_min": LaunchConfiguration("x_min"),
                    "x_max": LaunchConfiguration("x_max"),
                    "y_min": LaunchConfiguration("y_min"),
                    "y_max": LaunchConfiguration("y_max"),
                    "axis_margin": LaunchConfiguration("axis_margin"),
                    "use_boundary_bounds": LaunchConfiguration("use_boundary_bounds"),
                    "tick_step": LaunchConfiguration("tick_step"),
                },
            ],
        ),
        Node(
            package="tree_mapper",
            executable="tree_cluster_xy_plotter.py",
            name="tree_cluster_xy_plotter",
            output="screen",
            condition=IfCondition(LaunchConfiguration("run_cluster_plotter")),
            parameters=[
                config_yaml,
                {
                    "cluster_points_topic": "/vegetation_mapper/internal/cluster_points",
                    "cluster_labels_topic": "/vegetation_mapper/internal/cluster_labels",
                    "map_array_topic": "/tree_map_full",
                    "map_candidate_array_topic": "/tree_map_candidate_full",
                    "shrub_map_array_topic": "/shrub_map_full",
                    "shrub_map_candidate_array_topic": "/shrub_map_candidate_full",
                    "fixed_axes": LaunchConfiguration("fixed_axes"),
                    "x_min": LaunchConfiguration("x_min"),
                    "x_max": LaunchConfiguration("x_max"),
                    "y_min": LaunchConfiguration("y_min"),
                    "y_max": LaunchConfiguration("y_max"),
                    "axis_margin": LaunchConfiguration("axis_margin"),
                    "use_boundary_bounds": LaunchConfiguration("use_boundary_bounds"),
                    "tick_step": LaunchConfiguration("tick_step"),
                },
            ],
        ),
    ]

    return LaunchDescription(args + actions)
