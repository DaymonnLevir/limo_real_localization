# limo_bringup/launch/limo_slam.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # argumentos principais
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    # RViz config e launch do RTAB-Map
    pkg_share = FindPackageShare('limo_bringup')
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'rtabmap.rviz'])
    rtabmap_launch = PathJoinSubstitution([pkg_share, 'launch', 'rtabmap_rgbd_sync.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode.'
        ),

        # inclui o RTAB-Map RGBD Sync
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                'localization': localization,
                'use_sim_time': use_sim_time
            }.items()
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_file_name': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'scan_topic': 'scan',
                'odom_topic': 'odom',
            }]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
