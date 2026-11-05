# Solo escucha /odom y escribe: hora, x, y, yaw, v, w. No publica nada.
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
f = open(sys.argv[1], 'w', buffering=1)
f.write('hora,x,y,yaw,v,w\n')
def cb(m):
    q = m.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    t = time.time()
    f.write(f'{time.strftime("%H:%M:%S", time.localtime(t))}.{int(t%1*1000):03d},{m.pose.pose.position.x:.4f},'
            f'{m.pose.pose.position.y:.4f},{yaw:.5f},{m.twist.twist.linear.x:.3f},{m.twist.twist.angular.z:.3f}\n')
rclpy.init()
n = Node('log_odom_claude')
n.create_subscription(Odometry, '/odom', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
rclpy.spin(n)
