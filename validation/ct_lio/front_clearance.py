import json
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.create_node('ct_lio_front_clearance_check')
samples = []

def cb(msg):
    count = msg.width * msg.height
    fields = {f.name:f.offset for f in msg.fields}
    points = np.ndarray((count,), dtype=np.dtype({'names':['x','y','z'],
        'formats':['<f4']*3,'offsets':[fields['x'],fields['y'],fields['z']],
        'itemsize':msg.point_step}), buffer=msg.data)
    x,y,z = points['x'],points['y'],points['z']
    mask = (x>0.25)&(x<1.5)&(np.abs(y)<0.35)&(z>-.15)&(z<.8)
    samples.append({'corridor_point_count':int(mask.sum()),
        'minimum_x_m':float(x[mask].min()) if mask.any() else None})

sub = node.create_subscription(PointCloud2,'/livox/lidar',cb,qos_profile_sensor_data)
while len(samples)<10:
    rclpy.spin_once(node,timeout_sec=1)
print(json.dumps(samples,indent=2))
