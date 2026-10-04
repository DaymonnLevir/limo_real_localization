"""LIMO LiDAR-inertial estimator; the independent Livox driver runs separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fast_livo')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(package='fast_livo', executable='fastlivo_mapping',
             name='laserMapping', output='screen',
             parameters=[os.path.join(share, 'config', 'limo_mid360_only.yaml')]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', os.path.join(share, 'rviz_cfg', 'limo_mid360_only.rviz')]),
    ])
