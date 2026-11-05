# Solo escucha /scan: ajuste_puerta (como en fase1_node) con la pose medida con cinta como "creída".
# Uso: python3 puerta_en_vivo.py X Y THETA_GRADOS
import math, sys, yaml, os
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from autonomous_nav import comprobar_pose as cp
from autonomous_nav.fase1_node import ajuste_puerta
cfg = yaml.safe_load(open(os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav/config/fase1.yaml')))['fase1_node']['ros__parameters']
x, y, th = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
lista = []; ult = []
def cb(m):
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); ult[:] = [m]
rclpy.init(); n = Node('puerta_en_vivo_claude')
n.create_subscription(LaserScan, '/scan', cb, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
while rclpy.ok() and len(lista) < cfg['reloc_scans']: rclpy.spin_once(n, timeout_sec=1.0)
m = ult[0]; rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
lx, ly = x + cfg['lidar_offset_x'] * math.cos(th), y + cfg['lidar_offset_x'] * math.sin(th)
ok = ~np.isnan(rr)
px, py = lx + rr[ok] * np.cos(th + ang[ok]), ly + rr[ok] * np.sin(th + ang[ok])
dx, dy, dth, motivo, d = ajuste_puerta(px, py, x, y, cfg['reloc_wall_x'], cfg['reloc_door_y'], cfg['reloc_door_width'],
    cfg['reloc_door_width_tol'], cfg['reloc_band'], cfg['reloc_wall_length'], cfg['reloc_max_rms'], cfg['reloc_min_points'],
    cfg['reloc_max_dist'], math.radians(cfg['reloc_max_angle_deg']))
print(f'pose de cinta ({x:.3f}, {y:.3f}, {math.degrees(th):.1f}°)')
print(f'dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}°  {d["n"]} puntos, rms {d["rms"]*100 if d["rms"] is not None else float("nan"):.2f} cm  -> {"ACEPTADA" if motivo == "" else "RECHAZADA " + motivo}')
if d.get('y1') is not None:
    print(f'hueco medido {d["ancho"]:.3f} m: borde cercano {d["y1"]:.3f} (mapa 11.27, dif {d["y1"]-11.27:+.3f}), '
          f'borde lejano {d["y2"]:.3f} (mapa 12.02, dif {d["y2"]-12.02:+.3f})  [Y de los bordes con la pose de cinta]')
