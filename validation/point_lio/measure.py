"""Read-only wall-clock sensor/estimator validation; JSON contains compact evidence."""
import argparse
import json
import math
import time
import statistics
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry, Path
from livox_ros_driver2.msg import CustomMsg
from tf2_msgs.msg import TFMessage
from rclpy.serialization import deserialize_message

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=20)
p.add_argument('--output', required=True)
p.add_argument('--cloud-output', help='Optional .npz snapshot for geometric inspection')
a = p.parse_args()
rclpy.init()
n = rclpy.create_node('point_lio_validation')
data = {}
poses = []
acc = []
gyro = []
lidar = []
cloud = []
cloud_bounds = []
cloud_snapshot = None
frames = set()
invalid = 0
resets = {}
last = {}

def receive(topic, m):
    global invalid, cloud_snapshot
    t = time.monotonic()
    data.setdefault(topic, []).append(t)
    if isinstance(m, bytes):
        # CustomMsg has ~20k Python objects/frame. Decode once, count raw CDR
        # thereafter so measurement overhead does not fake a low sensor rate.
        if lidar:
            return
        m = deserialize_message(m, CustomMsg)
    if hasattr(m, 'header'):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if topic in last and stamp < last[topic]:
            resets[topic] = resets.get(topic, 0) + 1
        last[topic] = stamp
    if isinstance(m, Imu):
        av = [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]
        gv = [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        acc.append(math.sqrt(sum(v*v for v in av)))
        gyro.append(math.sqrt(sum(v*v for v in gv)))
        invalid += sum(not math.isfinite(v) for v in av + gv)
    elif isinstance(m, CustomMsg):
        offsets = [v.offset_time for v in m.points]
        lidar.append([m.point_num, min(offsets, default=0), max(offsets, default=0)])
    elif isinstance(m, Odometry):
        pos = m.pose.pose.position
        q = m.pose.pose.orientation
        vals = [pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w]
        invalid += sum(not math.isfinite(v) for v in vals)
        poses.append([t] + vals)
        frames.add((m.header.frame_id, m.child_frame_id))
    elif isinstance(m, PointCloud2):
        cloud.append([m.width*m.height, m.header.frame_id])
        fields = {v.name:v.offset for v in m.fields}
        view = np.ndarray((m.width*m.height,), dtype=np.dtype({
            'names':['x','y','z'], 'formats':['>f4' if m.is_bigendian else '<f4']*3,
            'offsets':[fields[v] for v in ['x','y','z']], 'itemsize':m.point_step}), buffer=m.data)
        xyz = np.column_stack([view[v] for v in ['x','y','z']])
        invalid += int((~np.isfinite(xyz)).sum())
        if len(xyz):
            cloud_bounds.append([xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()])
            cloud_snapshot = xyz
    elif isinstance(m, TFMessage):
        frames.update((v.header.frame_id, v.child_frame_id) for v in m.transforms)

subscriptions = []
for topic, typ in [('/livox/lidar', CustomMsg), ('/livox/imu', Imu),
                   ('/aft_mapped_to_init', Odometry), ('/cloud_registered', PointCloud2),
                   ('/path', Path), ('/tf', TFMessage)]:
    subscriptions.append(n.create_subscription(typ, topic, lambda m, t=topic: receive(t, m), qos_profile_sensor_data, raw=(typ is CustomMsg)))
start = time.monotonic()
while time.monotonic() - start < a.seconds:
    rclpy.spin_once(n, timeout_sec=0.1)
result = {'duration_s': time.monotonic()-start, 'topics': {}, 'invalid_values': invalid,
          'timestamp_regressions': resets, 'frames': sorted(frames)}
for topic, ts in data.items():
    result['topics'][topic] = {'count': len(ts), 'hz': (len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0,
                             'max_gap_s': max((y-x for x,y in zip(ts,ts[1:])), default=0)}
for key, values in [('acceleration_norm',acc),('gyro_norm',gyro)]:
    if values:
        result[key] = {'mean':statistics.mean(values),'std':statistics.pstdev(values),'min':min(values),'max':max(values)}
if lidar:
    result['lidar'] = {'mean_points':statistics.mean(v[0] for v in lidar), 'first_sample':lidar[0],
                       'min_points':min(v[0] for v in lidar)}
if cloud:
    result['registered_cloud'] = {'min_points':min(v[0] for v in cloud), 'max_points':max(v[0] for v in cloud), 'frame':cloud[-1][1]}
    result['registered_cloud']['last_bounds_m'] = cloud_bounds[-1] if cloud_bounds else []
if a.cloud_output and cloud_snapshot is not None:
    np.savez_compressed(a.cloud_output, xyz=cloud_snapshot)
if poses:
    origin = poses[0][1:4]
    d = lambda v: math.sqrt(sum((x-y)**2 for x,y in zip(v[1:4],origin)))
    result['odometry'] = {'first':poses[0], 'last':poses[-1], 'max_displacement_m':max(map(d,poses)),
                          'final_displacement_m':d(poses[-1]), 'samples':poses[::max(1,len(poses)//120)]}
with open(a.output,'w') as f:
    json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k!='odometry'},indent=2))
if poses:
    print(json.dumps({k:v for k,v in result['odometry'].items() if k!='samples'},indent=2))
n.destroy_node()
rclpy.shutdown()
