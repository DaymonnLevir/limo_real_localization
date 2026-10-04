import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Common parameters for all RTAB-Map nodes
    parameters=[{
          # Set frame_id to the robot's base frame
          'frame_id':'base_link',
          
          # Use topics with separate depth and color images
          'subscribe_depth':True,
          
          # Set to true to use the IMU's orientation to establish the map's gravity direction
          'wait_imu_to_init':True,
          
          # This is critical for cameras that are not hardware-synchronized.
          # It allows RTAB-Map to sync topics with slightly different timestamps.
          'approx_sync':True,
          
          # QoS settings for reliability
          # 1=Reliable, 2=Best Effort
          'qos_image_reliability':2,
          'qos_depth_reliability':2,
          'qos_camera_info_reliability':2
    }]

    # Remappings for all RTAB-Map nodes to match your Orbbec and LIMO topics
    remappings=[
          # Use the IMU from the robot
          ('imu', '/imu'),
          
          # Standard camera topic remappings
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw'),
          ('depth/camera_info', '/camera/depth/camera_info')
    ]

    return LaunchDescription([
        # RGBD Odometry Node
        # This node computes visual odometry from the camera data.
        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            remappings=remappings),

        # SLAM Node
        # This node builds the map, detects loop closures, and corrects the odometry.
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']), # '-d' deletes the previous database on start

        # Visualization Node
        # This node launches the RTAB-Map visualization GUI.
        # Node(
        #     package='rtabmap_viz', 
        #     executable='rtabmap_viz', 
        #     output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
    ])
