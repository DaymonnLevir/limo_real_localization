import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_RUN_ID = time.strftime("run_%Y%m%d_%H%M%S", time.localtime())


def generate_launch_description():
    args = [
        DeclareLaunchArgument("cluster_points_topic", default_value="/tree_mapper/internal/cluster_points"),
        DeclareLaunchArgument("cluster_labels_topic", default_value="/tree_mapper/internal/cluster_labels"),
        DeclareLaunchArgument("map_array_topic", default_value="/tree_map_full"),
        DeclareLaunchArgument("map_candidate_array_topic", default_value="/tree_map_candidate_full"),
        DeclareLaunchArgument("route_pose_topic", default_value=""),
        DeclareLaunchArgument("fixed_axes", default_value="false"),
        DeclareLaunchArgument("x_min", default_value="-10.0"),
        DeclareLaunchArgument("x_max", default_value="10.0"),
        DeclareLaunchArgument("y_min", default_value="-8.0"),
        DeclareLaunchArgument("y_max", default_value="7.0"),
        DeclareLaunchArgument("axis_margin", default_value="5.0"),
        DeclareLaunchArgument("use_boundary_bounds", default_value="false"),
        DeclareLaunchArgument("tick_step", default_value="5.0"),
        DeclareLaunchArgument("cluster_point_size", default_value="12.0"),
        DeclareLaunchArgument("cluster_alpha", default_value="0.62"),
        DeclareLaunchArgument("show_route_trace", default_value="true"),
        DeclareLaunchArgument("show_ground_truth", default_value="false"),
        DeclareLaunchArgument("ground_truth_csv_path", default_value=""),
        DeclareLaunchArgument("show_ground_truth_labels", default_value="true"),
        DeclareLaunchArgument("run_output_dir", default_value=""),
        DeclareLaunchArgument("run_id", default_value=DEFAULT_RUN_ID),
        DeclareLaunchArgument("runtime_state_namespace", default_value="/tree_mapper_runtime"),
        DeclareLaunchArgument("tree_map_export_now_topic", default_value="/tree_map_fuser/export_now"),
        DeclareLaunchArgument("tree_map_csv_output_path", default_value=""),
        DeclareLaunchArgument("tree_map_json_output_path", default_value=""),
        DeclareLaunchArgument("tree_map_history_output_path", default_value=""),
        DeclareLaunchArgument("tree_detection_history_output_path", default_value=""),
        DeclareLaunchArgument("snapshot_topic", default_value="~/save_snapshot"),
        DeclareLaunchArgument("snapshot_prefix", default_value="tree_snapshot"),
        DeclareLaunchArgument("save_on_shutdown", default_value="true"),
        DeclareLaunchArgument("shutdown_snapshot_basename", default_value="tree_cluster_state_final"),
    ]

    node = Node(
        package="tree_mapper",
        executable="tree_snapshot_exporter.py",
        name="tree_snapshot_exporter",
        output="screen",
        parameters=[
            {
                "cluster_points_topic": LaunchConfiguration("cluster_points_topic"),
                "cluster_labels_topic": LaunchConfiguration("cluster_labels_topic"),
                "map_array_topic": LaunchConfiguration("map_array_topic"),
                "map_candidate_array_topic": LaunchConfiguration("map_candidate_array_topic"),
                "route_pose_topic": LaunchConfiguration("route_pose_topic"),
                "fixed_axes": LaunchConfiguration("fixed_axes"),
                "x_min": LaunchConfiguration("x_min"),
                "x_max": LaunchConfiguration("x_max"),
                "y_min": LaunchConfiguration("y_min"),
                "y_max": LaunchConfiguration("y_max"),
                "axis_margin": LaunchConfiguration("axis_margin"),
                "use_boundary_bounds": LaunchConfiguration("use_boundary_bounds"),
                "tick_step": LaunchConfiguration("tick_step"),
                "cluster_point_size": LaunchConfiguration("cluster_point_size"),
                "cluster_alpha": LaunchConfiguration("cluster_alpha"),
                "show_route_trace": LaunchConfiguration("show_route_trace"),
                "show_ground_truth": LaunchConfiguration("show_ground_truth"),
                "ground_truth_csv_path": LaunchConfiguration("ground_truth_csv_path"),
                "show_ground_truth_labels": LaunchConfiguration("show_ground_truth_labels"),
                "run_output_dir": LaunchConfiguration("run_output_dir"),
                "run_id": LaunchConfiguration("run_id"),
                "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
                "tree_map_export_now_topic": LaunchConfiguration("tree_map_export_now_topic"),
                "tree_map_csv_output_path": LaunchConfiguration("tree_map_csv_output_path"),
                "tree_map_json_output_path": LaunchConfiguration("tree_map_json_output_path"),
                "tree_map_history_output_path": LaunchConfiguration("tree_map_history_output_path"),
                "tree_detection_history_output_path": LaunchConfiguration("tree_detection_history_output_path"),
                "snapshot_topic": LaunchConfiguration("snapshot_topic"),
                "snapshot_prefix": LaunchConfiguration("snapshot_prefix"),
                "save_on_shutdown": LaunchConfiguration("save_on_shutdown"),
                "shutdown_snapshot_basename": LaunchConfiguration("shutdown_snapshot_basename"),
            }
        ],
    )

    return LaunchDescription(args + [node])
