"""Lightweight receive-rate probe using raw CDR bytes for all subscribed types."""
import argparse
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry, Path

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=15)
p.add_argument('--output', required=True)
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('point_lio_rate_probe')
topics = [('/livox/lidar',CustomMsg),('/livox/imu',Imu),
          ('/aft_mapped_to_init',Odometry),('/cloud_registered',PointCloud2),('/path',Path)]
times = {name:[] for name,_ in topics}
subs=[]
for name,typ in topics:
    subs.append(n.create_subscription(typ,name,lambda _,key=name:times[key].append(time.monotonic()),qos_profile_sensor_data,raw=True))
t0=time.monotonic()
while time.monotonic()-t0<a.seconds:
    rclpy.spin_once(n,timeout_sec=0.1)
result={name:{'count':len(ts),'hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0}
        for name,ts in times.items()}
with open(a.output,'w') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
n.destroy_node()
rclpy.shutdown()
