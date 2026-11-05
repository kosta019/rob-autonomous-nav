# Offline: ajuste_puerta (como en fase1_node) con 10 scans del bag, robot parado delante de la puerta (pose creída del log)
import math, sys, yaml, os
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
from autonomous_nav import comprobar_pose as cp
from autonomous_nav.fase1_node import ajuste_puerta
cfg = yaml.safe_load(open(os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav/config/fase1.yaml')))['fase1_node']['ros__parameters']
x, y, th = 5.696, 11.017, 1.827
t0 = datetime.strptime('2026-11-05 ' + sys.argv[2], '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
lista = []
while r.has_next() and len(lista) < cfg['reloc_scans']:
    _, data, t = r.read_next()
    if t < t0: continue
    m = deserialize_message(data, LaserScan)
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a)
rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
lx, ly = x + cfg['lidar_offset_x'] * math.cos(th), y + cfg['lidar_offset_x'] * math.sin(th)
ok = ~np.isnan(rr)
px, py = lx + rr[ok] * np.cos(th + cfg['lidar_yaw_offset'] + ang[ok]), ly + rr[ok] * np.sin(th + cfg['lidar_yaw_offset'] + ang[ok])
dx, dy, dth, nivel, motivo, d = ajuste_puerta(px, py, x, y, cfg['reloc_wall_x'], cfg['reloc_door_y'], cfg['reloc_door_width'],
    cfg['reloc_door_width_tol'], cfg['reloc_band'], cfg['reloc_wall_length'], cfg['reloc_max_rms'], cfg['reloc_min_points'], cfg['reloc_min_points_near'],
    cfg['reloc_max_dist'], math.radians(cfg['reloc_max_angle_deg']))
rms = f"{d['rms']*100:.2f} cm" if d['rms'] is not None else '-'
print(f"{sys.argv[2]}: {nivel or 'RECHAZADA'} {motivo} dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}°  {d['n']} puntos, rms {rms}, "
      f"borde cercano {d['y1']} lejano {d['y2']} ancho {d['ancho']} aviso {d['aviso_ancho']}")
