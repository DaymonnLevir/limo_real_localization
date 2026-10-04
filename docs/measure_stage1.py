#!/usr/bin/env python3
"""Read-only measurement of real ROS topics; never publishes or moves the robot."""
import argparse, datetime, json, math, signal, struct, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, Path
p=argparse.ArgumentParser(); p.add_argument('--seconds',type=float,default=20); p.add_argument('--topics',nargs='*',default=None); args=p.parse_args()
rclpy.init(); node=rclpy.create_node('stage1_observer')
records={}; subscriptions=[]
stop_requested=False
def request_stop(signum, frame):
    global stop_requested
    stop_requested=True
signal.signal(signal.SIGINT,request_stop)
signal.signal(signal.SIGTERM,request_stop)
def stamp(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

def callback(topic,msg):
    now=time.monotonic(); rec=records.setdefault(topic,{'count':0,'first_receive':now,'first_stamp':stamp(msg),'nonmonotonic_stamps':0})
    if rec['count']:
        rec['max_receive_gap_seconds']=max(rec.get('max_receive_gap_seconds',0.0),now-rec['last_receive'])
        if stamp(msg)<=rec['last_stamp']: rec['nonmonotonic_stamps']+=1
    rec.update(count=rec['count']+1,last_receive=now,last_stamp=stamp(msg))
    if isinstance(msg,PointCloud2):
        rec.update(frame=msg.header.frame_id,points=msg.width*msg.height,fields=[{'name':f.name,'datatype':f.datatype,'offset':f.offset} for f in msg.fields],point_step=msg.point_step)
        fields={f.name:f for f in msg.fields}
        if msg.width and 'timestamp' in fields and fields['timestamp'].datatype==8:
            endian='>' if msg.is_bigendian else '<'
            first=struct.unpack_from(endian+'d',msg.data,fields['timestamp'].offset)[0]
            last=struct.unpack_from(endian+'d',msg.data,(msg.width*msg.height-1)*msg.point_step+fields['timestamp'].offset)[0]
            rec.update(point_timestamp_first=first,point_timestamp_last=last,point_minus_header_seconds=first*1e-9-stamp(msg),scan_seconds=(last-first)*1e-9)
    if isinstance(msg,Imu):
        rec.update(acceleration=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    if isinstance(msg,Odometry):
        pos=msg.pose.pose.position; q=msg.pose.pose.orientation
        pose=[pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        rec['last_yaw_rad']=yaw
        rec.setdefault('first_yaw_rad',yaw)
        rec['yaw_min_rad']=min(rec.get('yaw_min_rad',yaw),yaw)
        rec['yaw_max_rad']=max(rec.get('yaw_max_rad',yaw),yaw)
        if 'first_pose' not in rec:
            rec.update(first_pose=pose,position_min=pose[:3].copy(),position_max=pose[:3].copy(),max_translation_from_first=0.0)
        if 'last_pose' in rec:
            step=math.dist(rec['last_pose'][:3],pose[:3])
            rec['max_position_step_m']=max(rec.get('max_position_step_m',0.0),step)
            rec['accumulated_translation_m']=rec.get('accumulated_translation_m',0.0)+step
        if now-rec.get('last_trace_receive',0)>1.0:
            rec.setdefault('pose_trace',[]).append({'stamp':stamp(msg),'pose':pose,'yaw':yaw})
            rec['last_trace_receive']=now
        rec.update(last_pose=pose,frame=msg.header.frame_id,child_frame=msg.child_frame_id)
        rec['position_min']=[min(a,b) for a,b in zip(rec['position_min'],pose[:3])]
        rec['position_max']=[max(a,b) for a,b in zip(rec['position_max'],pose[:3])]
        rec['max_translation_from_first']=max(rec['max_translation_from_first'],math.dist(rec['first_pose'][:3],pose[:3]))
        rec['all_finite']=rec.get('all_finite',True) and all(math.isfinite(x) for x in pose)
    if isinstance(msg,Path): rec.update(poses=len(msg.poses),frame=msg.header.frame_id)

for topic,kind in [('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/aft_mapped_to_init',Odometry),('/path',Path),('/cloud_registered',PointCloud2)]:
    if args.topics is None or topic in args.topics:
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,t=topic:callback(t,msg),qos_profile_sensor_data))
start=time.monotonic()
while not stop_requested and time.monotonic()-start<args.seconds: rclpy.spin_once(node,timeout_sec=0.1)
for topic,rec in records.items():
    rec['hz']=(rec['count']-1)/max(1e-9,rec['last_receive']-rec['first_receive'])
    rec['publishers']=[{'node':x.node_name,'type':x.topic_type} for x in node.get_publishers_info_by_topic(topic)]
    rec['subscribers']=[{'node':x.node_name,'type':x.topic_type} for x in node.get_subscriptions_info_by_topic(topic)]
print(json.dumps({'ended_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'duration_seconds':time.monotonic()-start,'topics':records},indent=2))
node.destroy_node(); rclpy.shutdown()
