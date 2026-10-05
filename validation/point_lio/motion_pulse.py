import time
import rclpy
from geometry_msgs.msg import Twist
rclpy.init(); n=rclpy.create_node('point_lio_bounded_pulse'); p=n.create_publisher(Twist,'/point_lio_test/lease',10)
try:
    t=time.monotonic()
    while time.monotonic()-t<2:rclpy.spin_once(n,timeout_sec=.02);p.publish(Twist())
    if p.get_subscription_count()!=1:raise RuntimeError('Expected exactly one motion guard')
    t=time.monotonic()
    while time.monotonic()-t<2.0:
        m=Twist();m.linear.x=.02;p.publish(m);rclpy.spin_once(n,timeout_sec=.02)
finally:
    for _ in range(100):p.publish(Twist());time.sleep(.02)
    n.destroy_node();rclpy.shutdown()
