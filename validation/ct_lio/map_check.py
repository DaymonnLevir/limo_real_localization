#!/usr/bin/env python3
import json
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.create_node('ct_lio_map_check')
result = {}

def callback(msg):
    count = msg.width * msg.height
    fields = {f.name:f.offset for f in msg.fields}
    if count and all(name in fields for name in ('x','y','z')):
        values = np.ndarray((count,), dtype=np.dtype({'names':['x','y','z'],
            'formats':['<f4']*3,'offsets':[fields['x'],fields['y'],fields['z']],
            'itemsize':msg.point_step}), buffer=msg.data)
        result.update({'frame_id':msg.header.frame_id,'points':count,
            'nonfinite_xyz_points':int((~(np.isfinite(values['x']) & np.isfinite(values['y']) & np.isfinite(values['z']))).sum()),
            'min_xyz_m':[float(values[k].min()) for k in ('x','y','z')],
            'max_xyz_m':[float(values[k].max()) for k in ('x','y','z')]})

sub = node.create_subscription(PointCloud2,'/ct_lio/map_incremental',callback,qos_profile_sensor_data)
while not result:
    rclpy.spin_once(node,timeout_sec=1)
print(json.dumps(result,indent=2))
