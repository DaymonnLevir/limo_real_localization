#!/usr/bin/env python3
"""Count wire messages without deserializing large clouds or paths."""
import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=10)
p.add_argument('--output', required=True)
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('ct_lio_raw_rate_probe')
topics = [('/livox/lidar', PointCloud2), ('/livox/imu', Imu),
          ('/ct_lio/odom', Odometry), ('/ct_lio/odometry_path', Path),
          ('/ct_lio/map_incremental', PointCloud2), ('/tf', TFMessage)]
times = {key: [] for key, _ in topics}
subs = [n.create_subscription(kind, key, lambda _, name=key: times[name].append(time.monotonic()),
        qos_profile_sensor_data, raw=True) for key, kind in topics]
start = time.monotonic()
while time.monotonic() - start < a.seconds:
    rclpy.spin_once(n, timeout_sec=.02)
result = {key: {'count': len(ts), 'hz': (len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0}
          for key, ts in times.items()}
with open(a.output, 'w') as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
subs.clear()
n.destroy_node()
rclpy.shutdown()
