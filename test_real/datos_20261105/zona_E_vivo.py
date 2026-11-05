# Solo escucha /scan: con la pose creída del robot, puntos alrededor de E (X 4.4-5.95, Y 5.2-6.8), agrupados
import math, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from autonomous_nav import comprobar_pose as cp
x, y, th = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
lista = []; ult = []
def cb(m):
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); ult[:] = [m]
rclpy.init(); n = Node('zona_E_vivo_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(lista) < 5: rclpy.spin_once(n, timeout_sec=1.0)
m = ult[0]; rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
ok = ~np.isnan(rr)
px, py = lx + rr[ok] * np.cos(th + ang[ok]), ly + rr[ok] * np.sin(th + ang[ok])
sel = (px > 4.4) & (px < 5.95) & (py > 5.2) & (py < 6.8)
pts = sorted(zip(px[sel], py[sel]), key=lambda q: (round(q[0], 1), q[1]))
grupos = []
for X, Y in sorted(zip(px[sel], py[sel]), key=lambda q: q[1]):
    if grupos and math.hypot(X - grupos[-1][-1][0], Y - grupos[-1][-1][1]) < 0.08: grupos[-1].append((X, Y))
    else: grupos.append([(X, Y)])
print('E = (5.082, 6.122); mesa pequeña (zona): X 5.51-5.90, Y 5.57-6.58')
for g in grupos:
    gx = [p[0] for p in g]; gy = [p[1] for p in g]
    print(f'  grupo de {len(g):2d} puntos: X {min(gx):.2f}-{max(gx):.2f}, Y {min(gy):.2f}-{max(gy):.2f}  (a {math.hypot(np.mean(gx)-5.082, np.mean(gy)-6.122):.2f} m de E)')
