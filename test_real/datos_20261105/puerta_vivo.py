# Solo escucha /scan: con la pose creída del robot, puntos cerca de la puerta (X 5.9-6.6, Y 11.0-12.3)
import math, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
x, y, th = 5.696, 11.017, 1.827
lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
m = []
rclpy.init(); n = Node('puerta_vivo_claude')
n.create_subscription(LaserScan, '/scan', lambda s: m.append(s), QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(m) < 5: rclpy.spin_once(n, timeout_sec=1.0)
s = m[-1]
rr = np.array(s.ranges); a = s.angle_min + np.arange(len(rr)) * s.angle_increment
ok = np.isfinite(rr) & (rr >= s.range_min)
px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
sel = (px > 5.9) & (px < 6.7) & (py > 11.0) & (py < 12.3)
for X, Y, R in sorted(zip(px[sel], py[sel], rr[ok][sel]), key=lambda q: q[1]):
    print(f'  ({X:.2f}, {Y:.2f})  a {R:.2f} m')
print('Mapa: pared de la sala X 6.34-6.39; hueco de la puerta Y 11.27-12.02; Door = (6.49, 11.68)')
