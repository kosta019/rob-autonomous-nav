# Solo escucha /scan: puntos en el mundo con la pose creída, resumidos por ángulo (cada 10°)
import math, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from autonomous_nav import comprobar_pose as cp
x, y, th = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
lista = []; ult = []
def cb(m):
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); ult[:] = [m]
rclpy.init(); n = Node('escaneo_mundo_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(lista) < 5: rclpy.spin_once(n, timeout_sec=1.0)
m = ult[0]; rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
for k in range(0, 360, 10):
    i = k
    if np.isnan(rr[i]): print(f'  rayo {k:3d}° (mundo {math.degrees(th + ang[i]) % 360:5.1f}°): sin dato'); continue
    px, py = lx + rr[i] * math.cos(th + ang[i]), ly + rr[i] * math.sin(th + ang[i])
    print(f'  rayo {k:3d}° (mundo {math.degrees(th + ang[i]) % 360:5.1f}°): {rr[i]:.2f} m -> ({px:.2f}, {py:.2f})')
