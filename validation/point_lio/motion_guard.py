# Temporary, bounded physical-validation guard. It never forwards commands without live feedback.
import json,math,signal,time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from limo_msgs.msg import LimoStatus
rclpy.init(); n=rclpy.create_node('point_lio_motion_guard'); out=n.create_publisher(Twist,'/point_lio_test/cmd_vel',10)
seen={}; data={}; desired=0.; first=None; started=None; latched=None; running=True; active_time=0.; last_tick=time.monotonic()
def status(m):
    seen['status']=time.monotonic();data['status']={'vehicle_state':m.vehicle_state,'control_mode':m.control_mode,'motion_mode':m.motion_mode,'error_code':m.error_code,'battery_voltage':m.battery_voltage}
def wheel(m):
    seen['wheel']=time.monotonic();data['wheel_speed']=[m.twist.twist.linear.x,m.twist.twist.angular.z]
def pose(m):
    global first
    seen['pose']=time.monotonic(); p=m.pose.pose.position; q=m.pose.pose.orientation
    data['pose']=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
    if first is None:first=data['pose'][:3].copy()
def cloud(m):
    if 'pose' not in data:return
    a=np.ndarray((m.width*m.height,),dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[0,4,8],'itemsize':m.point_step}),buffer=m.data)
    # Registered cloud is in camera_init. Bring it back to the IMU frame
    # before checking radial clearance; this is only a test guard.
    p=data['pose'];x,y,z,w=p[3:]
    rot=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    xyz=(np.column_stack((a['x'],a['y'],a['z']))-p[:3])@rot
    r=np.hypot(xyz[:,0],xyz[:,1]);mask=(r>.20)&(xyz[:,2]>-.25)&(xyz[:,2]<1.2)&np.isfinite(r)
    if int(mask.sum())<50:return
    data['clearance']=float(r[mask].min()); seen['lidar']=time.monotonic()
def lease(m):
    global desired
    seen['lease']=time.monotonic()
    # Forward only a small straight movement. No reverse, lateral motion, or turn.
    desired=max(0.,min(.02,m.linear.x)) if abs(m.angular.z)<1e-9 else 0.
def halt(sig,frame):
    global running
    running=False
signal.signal(signal.SIGINT,halt);signal.signal(signal.SIGTERM,halt)
subs=[n.create_subscription(LimoStatus,'/point_lio_test/status',status,10),n.create_subscription(Odometry,'/point_lio_test/wheel_odom',wheel,qos_profile_sensor_data),n.create_subscription(Odometry,'/aft_mapped_to_init',pose,qos_profile_sensor_data),n.create_subscription(PointCloud2,'/cloud_registered',cloud,qos_profile_sensor_data),n.create_subscription(Twist,'/point_lio_test/lease',lease,10)]
t0=time.monotonic(); last_report=0
try:
    while running and time.monotonic()-t0<300:
        rclpy.spin_once(n,timeout_sec=.01);now=time.monotonic();dt=now-last_tick;last_tick=now
        reason=None
        for key in ['status','wheel','pose','lidar']:
            limit=1.0 if key=='status' else .35
            if now-seen.get(key,0)>limit:reason='stale_'+key;break
        if reason is None:
            st=data['status']
            if st['motion_mode'] not in (0,2) or st['control_mode']!=1 or st['vehicle_state']!=0 or st['error_code']!=0:reason='base_state'
            elif not all(math.isfinite(x) for x in data['pose']):reason='invalid_pose'
            elif math.dist(first,data['pose'][:3])>=.07:reason='distance_limit'
            elif data['clearance']<.55:reason='obstacle'
            elif abs(data['wheel_speed'][0])>.05 or abs(data['wheel_speed'][1])>.1:reason='unexpected_speed'
            elif active_time>=3.0:reason='time_limit'
        wanted=desired>0 and now-seen.get('lease',0)<.12
        if started is not None and reason is not None:latched=reason
        msg=Twist()
        if wanted and reason is None and latched is None:
            if started is None:started=now
            active_time+=dt;msg.linear.x=desired
        out.publish(msg)
        if now-last_report>1:
            last_report=now
            print(json.dumps({'elapsed':now-t0,'ready':reason is None and latched is None,'reason':reason,'latched':latched,'active_seconds':active_time,'output_v':msg.linear.x,**data}),flush=True)
finally:
    for _ in range(50):out.publish(Twist());time.sleep(.02)
    print(json.dumps({'stopped':True,'active_seconds':active_time,'latched':latched,**data}),flush=True)
    n.destroy_node();rclpy.shutdown()
