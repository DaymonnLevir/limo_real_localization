"""ct_lio_ros2 启动文件: 加载参数文件并启动里程计节点 + rviz 可视化.
支持 bag 回放时切换模拟时钟(use_sim_time:=true + --clock)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ct_lio_ros2')

    # 参数文件(可覆盖)
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'params.yaml'),
        description='Path to params.yaml',
    )

    # use_sim_time: bag 回放时须设 true(节点/TF/rviz 在模拟时钟下, 数据时间戳才能对齐)
    #   真机实时默认 false(避免等待 /clock); 回放时 ros2 launch ct_lio_ros2 ct_lio.launch.py use_sim_time:=true
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true for rosbag playback)',
    )

    # rviz 配置文件(可覆盖)
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'config', 'rviz', 'default.rviz'),
        description='Path to rviz config file',
    )

    lio_node = Node(
        package='ct_lio_ros2',
        executable='ct_lio_node',
        name='ct_lio_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # rviz2: 加载预置可视化(Fixed Frame=odom, 轨迹/增量地图)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        params_file_arg,
        sim_time_arg,
        rviz_config_arg,
        lio_node,
        rviz_node,
    ])
