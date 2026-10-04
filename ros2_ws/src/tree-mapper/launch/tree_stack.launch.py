import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("tree_mapper")
    runtime_launch = os.path.join(pkg_share, "launch", "tree_stack_runtime.launch.py")
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
        DeclareLaunchArgument("run_id", default_value=""),
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

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(runtime_launch),
        launch_arguments={
            "config_yaml": LaunchConfiguration("config_yaml"),
            "run_plotter": LaunchConfiguration("run_plotter"),
            "run_cluster_plotter": LaunchConfiguration("run_cluster_plotter"),
            "run_fuser": LaunchConfiguration("run_fuser"),
            "run_snapshot_exporter": LaunchConfiguration("run_snapshot_exporter"),
            "candidate_history_csv_output_path": LaunchConfiguration("candidate_history_csv_output_path"),
            "tree_fuser_csv_output_path": LaunchConfiguration("tree_fuser_csv_output_path"),
            "tree_fuser_json_output_path": LaunchConfiguration("tree_fuser_json_output_path"),
            "tree_fuser_history_csv_output_path": LaunchConfiguration("tree_fuser_history_csv_output_path"),
            "shrub_fuser_csv_output_path": LaunchConfiguration("shrub_fuser_csv_output_path"),
            "shrub_fuser_json_output_path": LaunchConfiguration("shrub_fuser_json_output_path"),
            "shrub_fuser_history_csv_output_path": LaunchConfiguration("shrub_fuser_history_csv_output_path"),
            "run_output_dir": LaunchConfiguration("run_output_dir"),
            "run_id": LaunchConfiguration("run_id"),
            "runtime_state_namespace": LaunchConfiguration("runtime_state_namespace"),
            "fixed_axes": LaunchConfiguration("fixed_axes"),
            "x_min": LaunchConfiguration("x_min"),
            "x_max": LaunchConfiguration("x_max"),
            "y_min": LaunchConfiguration("y_min"),
            "y_max": LaunchConfiguration("y_max"),
            "axis_margin": LaunchConfiguration("axis_margin"),
            "use_boundary_bounds": LaunchConfiguration("use_boundary_bounds"),
            "tick_step": LaunchConfiguration("tick_step"),
        }.items(),
    )

    return LaunchDescription(args + [include])
