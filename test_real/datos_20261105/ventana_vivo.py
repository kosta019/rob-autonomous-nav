# Solo escucha /scan: puntos de la ventana del tramo cercano (banda 0.35 alrededor de x = 6.39, y de 9.97 a 11.67),
# en el mundo con la pose medida con cinta, mediana de 10 scans
import math, sys
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from autonomous_nav import comprobar_pose as cp
x, y, th = 5.96, 11.66, math.radians(90.0)
lista = []; ult = []
def cb(m):
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); ult[:] = [m]
rclpy.init(); n = Node('ventana_vivo_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(lista) < 10: rclpy.spin_once(n, timeout_sec=1.0)
m = ult[0]; rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
ok = ~np.isnan(rr)
px, py, ia = lx + rr[ok] * np.cos(th + ang[ok]), ly + rr[ok] * np.sin(th + ang[ok]), np.degrees(ang[ok])
sel = (np.abs(px - 6.39) < 0.35) & (py > 9.97) & (py < 11.67)
for X, Y, A in sorted(zip(px[sel], py[sel], ia[sel]), key=lambda q: q[1]):
    print(f'  y {Y:6.3f}  x {X:5.3f}  (rayo {A:5.1f}°)')
lej = (np.abs(px - 6.39) < 0.35) & (py >= 11.67) & (py < 13.0)
print('lado lejano (y >= 11.67):', [(round(float(a), 2), round(float(b), 2)) for a, b in sorted(zip(px[lej], py[lej]), key=lambda q: q[1])][:25])
