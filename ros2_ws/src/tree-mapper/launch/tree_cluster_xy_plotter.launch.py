from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("cluster_points_topic", default_value="/tree_detector_cluster_points"),
        DeclareLaunchArgument("cluster_labels_topic", default_value="/tree_detector_cluster_labels"),
        DeclareLaunchArgument("map_array_topic", default_value="/tree_map_full"),
        DeclareLaunchArgument("map_candidate_array_topic", default_value="/tree_map_candidate_full"),
        DeclareLaunchArgument("shrub_cluster_points_topic", default_value="/shrub_detector_cluster_points"),
        DeclareLaunchArgument("shrub_cluster_labels_topic", default_value="/shrub_detector_cluster_labels"),
        DeclareLaunchArgument("shrub_map_array_topic", default_value="/shrub_map_full"),
        DeclareLaunchArgument("shrub_map_candidate_array_topic", default_value="/shrub_map_candidate_full"),
        DeclareLaunchArgument("refresh_ms", default_value="500"),
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
        DeclareLaunchArgument("show_map_labels", default_value="true"),
        DeclareLaunchArgument("show_metrics", default_value="true"),
        DeclareLaunchArgument("max_labels", default_value="90"),
        DeclareLaunchArgument("show_route_trace", default_value="true"),
        DeclareLaunchArgument("route_pose_topic", default_value="/odom"),
        DeclareLaunchArgument("route_min_point_dist", default_value="0.15"),
        DeclareLaunchArgument("route_max_points", default_value="4000"),
        DeclareLaunchArgument("route_line_width", default_value="1.8"),
        DeclareLaunchArgument("ground_truth_csv_path", default_value=""),
    ]

    node = Node(
        package="tree_mapper",
        executable="tree_cluster_xy_plotter.py",
        name="tree_cluster_xy_plotter",
        output="screen",
        parameters=[
            {
                "cluster_points_topic": LaunchConfiguration("cluster_points_topic"),
                "cluster_labels_topic": LaunchConfiguration("cluster_labels_topic"),
                "map_array_topic": LaunchConfiguration("map_array_topic"),
                "map_candidate_array_topic": LaunchConfiguration("map_candidate_array_topic"),
                "shrub_cluster_points_topic": LaunchConfiguration("shrub_cluster_points_topic"),
                "shrub_cluster_labels_topic": LaunchConfiguration("shrub_cluster_labels_topic"),
                "shrub_map_array_topic": LaunchConfiguration("shrub_map_array_topic"),
                "shrub_map_candidate_array_topic": LaunchConfiguration("shrub_map_candidate_array_topic"),
                "refresh_ms": LaunchConfiguration("refresh_ms"),
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
                "show_map_labels": LaunchConfiguration("show_map_labels"),
                "show_metrics": LaunchConfiguration("show_metrics"),
                "max_labels": LaunchConfiguration("max_labels"),
                "show_route_trace": LaunchConfiguration("show_route_trace"),
                "route_pose_topic": LaunchConfiguration("route_pose_topic"),
                "route_min_point_dist": LaunchConfiguration("route_min_point_dist"),
                "route_max_points": LaunchConfiguration("route_max_points"),
                "route_line_width": LaunchConfiguration("route_line_width"),
                "ground_truth_csv_path": LaunchConfiguration("ground_truth_csv_path"),
            }
        ],
    )

    return LaunchDescription(args + [node])
