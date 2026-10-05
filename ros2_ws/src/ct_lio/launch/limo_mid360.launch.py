import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('ct_lio_ros2')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='ct_lio_ros2', executable='ct_lio_node', name='ct_lio_node',
             output='screen',
             parameters=[os.path.join(share, 'config', 'limo_mid360.yaml')]),
        Node(package='rviz2', executable='rviz2', name='ct_lio_rviz',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'config', 'rviz', 'limo_mid360.rviz')],
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
