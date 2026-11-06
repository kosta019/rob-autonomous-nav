# ¿Funcionaría la relocalización desde donde se bloqueó el robot (13:44, 0.21 m antes de E-Door_2)?
# Mediana de 10 scans, como el nodo (reloc_scans), en 5 ventanas, con ajuste_puerta y los parámetros del yaml.
import math, sys
import numpy as np, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import os
PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'autonomous_nav')
sys.path.insert(0, PKG)
from autonomous_nav.fase1_node import ajuste_puerta
from autonomous_nav import comprobar_pose as cp
P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
x, y, th = 5.938, 11.256, 1.472
t0, t1 = 1793969046.5, 1793969067.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
scans, s0 = [], None
while r.has_next():
    _, data, t = r.read_next()
    if t0 < t * 1e-9 < t1:
        s = deserialize_message(data, LaserScan); s0 = s
        rr = np.array(s.ranges, dtype=float)
        rr[~np.isfinite(rr) | (rr < s.range_min) | (rr > s.range_max)] = np.nan
        scans.append(rr)
a = s0.angle_min + np.arange(len(scans[0])) * s0.angle_increment
def ajuste(rr, x, y, th):
    lx, ly = x + P['lidar_offset_x'] * math.cos(th), y + P['lidar_offset_x'] * math.sin(th)
    ok = ~np.isnan(rr)
    px = lx + rr[ok] * np.cos(th + P['lidar_yaw_offset'] + a[ok]); py = ly + rr[ok] * np.sin(th + P['lidar_yaw_offset'] + a[ok])
    return ajuste_puerta(px, py, x, y, P['reloc_wall_x'], P['reloc_edge_y'], P['reloc_door_width'], P['reloc_door_width_tol'],
                         P['reloc_band'], P['reloc_wall_length'], P['reloc_max_rms'], P['reloc_min_points'], P['reloc_min_points_near'],
                         P['reloc_max_dx'], P['reloc_max_dy'], math.radians(P['reloc_max_angle_deg']), P['reloc_frame_depth'],
                         P['reloc_duct_end_y'])
n = P['reloc_scans']
for k in range(5):
    rr = cp.mediana_de_scans(scans[k * n:(k + 1) * n])
    dx, dy, dth, nivel, motivo, d = ajuste(rr, x, y, th)
    f = lambda v: '-' if v is None else f'{v:.3f}'
    linea = (f"{k+1}: {nivel or 'RECHAZADA'} {motivo}: dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}°  n {d['n']} "
             f"y_fin {f(d['y_fin'])} y1 (marco) {f(d['y1'])} ancho {f(d['ancho'])}")
    if nivel:
        nx, ny, nth = x + dx, (y + dy if nivel == 'total' else y), th + dth
        dx2, dy2, dth2, nivel2, motivo2, _ = ajuste(rr, nx, ny, nth)
        linea += f"  | repetido: {nivel2} dx={dx2:+.3f} dy={dy2:+.3f} dth={math.degrees(dth2):+.2f}°"
    print(linea)
