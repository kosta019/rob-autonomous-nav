# Solo escucha /imu unos segundos: media y máximo de la velocidad angular en x, y, z
import numpy as np, rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
w = []
rclpy.init(); n = Node('imu_media_claude')
n.create_subscription(Imu, '/imu', lambda m: w.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)), QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(w) < int(sys.argv[1]): rclpy.spin_once(n, timeout_sec=1.0)
w = np.array(w); print(f'{len(w)} mensajes /imu: media x {w[:,0].mean():+.4f} y {w[:,1].mean():+.4f} z {w[:,2].mean():+.4f} rad/s; |z| max {abs(w[:,2]).max():.4f}')
