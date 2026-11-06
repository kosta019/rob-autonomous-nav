# Solo escucha /scan: ajuste_puerta (versión con lateral del marco) con una pose creída dada, N repeticiones
# Uso: python3 puerta_vivo2.py X Y THETA_GRADOS [REPETICIONES]
import math, sys, yaml, os
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from autonomous_nav import comprobar_pose as cp
from autonomous_nav.fase1_node import ajuste_puerta
c = yaml.safe_load(open(os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav/config/fase1.yaml')))['fase1_node']['ros__parameters']
x, y, th = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
rep = int(sys.argv[4]) if len(sys.argv) > 4 else 1
lista = []; ult = []
def cb(m):
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); ult[:] = [m]
rclpy.init(); n = Node('puerta_vivo2_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
def ajuste(rr, ang, x, y, th):
    lx, ly = x + c['lidar_offset_x'] * math.cos(th), y + c['lidar_offset_x'] * math.sin(th)
    ok = ~np.isnan(rr)
    px, py = lx + rr[ok] * np.cos(th + ang[ok]), ly + rr[ok] * np.sin(th + ang[ok])
    return ajuste_puerta(px, py, x, y, c['reloc_wall_x'], c['reloc_edge_y'], c['reloc_door_width'], c['reloc_door_width_tol'],
                         c['reloc_band'], c['reloc_wall_length'], c['reloc_max_rms'], c['reloc_min_points'], c['reloc_min_points_near'],
                         c['reloc_max_dx'], c['reloc_max_dy'], math.radians(c['reloc_max_angle_deg']), c['reloc_frame_depth'])
print(f'pose creída ({x:.3f}, {y:.3f}, {math.degrees(th):.1f}°); referencias wall_x {c["reloc_wall_x"]} edge_y {c["reloc_edge_y"]}')
for k in range(rep):
    lista.clear()
    while rclpy.ok() and len(lista) < c['reloc_scans']: rclpy.spin_once(n, timeout_sec=1.0)
    m = ult[0]; rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
    dx, dy, dth, nivel, motivo, d = ajuste(rr, ang, x, y, th)
    f = lambda v, e=3: '-' if v is None else f'{v:.{e}f}'
    print(f"{k+1}: {nivel or 'RECHAZADA'} {motivo}: dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}°  n {d['n']} rms {f(d['rms'] and d['rms']*100, 2)} cm  "
          f"x_linea {f(d['x_linea'])}  y_fin (canaleta) {f(d['y_fin'])}  y1 (marco) {f(d['y1'])}  y2 {f(d['y2'])}  ancho {f(d['ancho'])} aviso {d['aviso_ancho']}")
    if nivel:
        nx, ny, nth = x + dx, (y + dy if nivel == 'total' else y), th + dth
        dx2, dy2, dth2, nivel2, motivo2, _ = ajuste(rr, ang, nx, ny, nth)
        print(f"   repetido desde la pose corregida: {nivel2 or 'RECHAZADA'} {motivo2} dx={dx2:+.3f} dy={dy2:+.3f} dth={math.degrees(dth2):+.2f}°")
