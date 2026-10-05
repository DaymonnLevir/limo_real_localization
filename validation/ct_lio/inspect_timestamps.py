import json
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = rclpy.create_node('ct_lio_timestamp_inspector')
frames = []

def cb(msg):
    field = next(f for f in msg.fields if f.name == 'timestamp')
    count = msg.width * msg.height
    values = np.ndarray((count,), dtype=np.dtype({'names':['t'], 'formats':['<f8'],
        'offsets':[field.offset], 'itemsize':msg.point_step}), buffer=msg.data)['t']
    diff = np.diff(values)
    negative = diff[diff < 0]
    frames.append({'count':count,'first':float(values[0]),'last':float(values[-1]),
        'min':float(values.min()),'max':float(values.max()),
        'negative_count':int(negative.size),
        'negative_min_ns':float(negative.min()) if negative.size else None,
        'negative_median_ns':float(np.median(negative)) if negative.size else None,
        'first_20_offsets_ms':((values[:20]-values[0])/1e6).tolist(),
        'minmax_span_s':float((values.max()-values.min())/1e9)})

sub = node.create_subscription(PointCloud2,'/livox/lidar',cb,qos_profile_sensor_data)
while len(frames)<3:
    rclpy.spin_once(node,timeout_sec=1)
print(json.dumps(frames,indent=2))
