#!/usr/bin/env python3
import json
import math
import rclpy
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node('ct_lio_path_check')
result = {}

def callback(message):
    points = [(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
              for pose in message.poses]
    segments = [math.dist(a, b) for a, b in zip(points[:-1], points[1:])]
    result.update({'frame_id':message.header.frame_id, 'poses':len(points),
                   'start_xyz_m':points[0] if points else None,
                   'end_xyz_m':points[-1] if points else None,
                   'largest_segment_m':max(segments) if segments else None,
                   'nonfinite_pose_count':sum(not all(map(math.isfinite, p)) for p in points)})

sub = node.create_subscription(Path,'/ct_lio/odometry_path',callback,qos_profile_sensor_data)
while not result:
    rclpy.spin_once(node,timeout_sec=1)
print(json.dumps(result,indent=2))
