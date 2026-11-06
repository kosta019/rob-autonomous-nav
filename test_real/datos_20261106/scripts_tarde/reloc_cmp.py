import math, sys, os
import numpy as np, yaml, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
PKG = os.path.expanduser('~/ros2_ws/src/rob-autonomous-nav/autonomous_nav'); sys.path.insert(0, PKG)
from autonomous_nav.fase1_node import ajuste_puerta
from autonomous_nav import comprobar_pose as cp
P = {}
for f in ('fase1.yaml', 'fase1_ruta.yaml', 'real.yaml'):
    P.update(yaml.safe_load(open(f'{PKG}/config/{f}'))['fase1_node']['ros__parameters'])
mcap, t0, t1, x, y, th = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), *map(float, sys.argv[4:7])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
scans = []
while r.has_next():
    _, data, t = r.read_next()
    if t0 < t * 1e-9 < t1:
        s = deserialize_message(data, LaserScan); s0 = s
        rr = np.array(s.ranges, dtype=float); rr[~np.isfinite(rr) | (rr < s.range_min) | (rr > s.range_max)] = np.nan
        scans.append(rr)
print(len(scans), 'scans')
a = s0.angle_min + np.arange(len(scans[0])) * s0.angle_increment
rr = cp.mediana_de_scans(scans[-P['reloc_scans']:])
lx, ly = x + P['lidar_offset_x'] * math.cos(th), y + P['lidar_offset_x'] * math.sin(th)
ok = ~np.isnan(rr)
px = lx + rr[ok] * np.cos(th + P['lidar_yaw_offset'] + a[ok]); py = ly + rr[ok] * np.sin(th + P['lidar_yaw_offset'] + a[ok])
res = ajuste_puerta(px, py, x, y, P['reloc_wall_x'], P['reloc_edge_y'], P['reloc_door_width'], P['reloc_door_width_tol'],
    P['reloc_band'], P['reloc_wall_length'], P['reloc_max_rms'], P['reloc_min_points'], P['reloc_min_points_near'],
    P['reloc_max_dx'], P['reloc_max_dy'], math.radians(P['reloc_max_angle_deg']), P['reloc_frame_depth'], P['reloc_duct_end_y'])
dx, dy, dth, nivel, motivo, d = res
print(nivel, motivo, f'dx={dx:+.3f} dy={dy:+.3f} dth={math.degrees(dth):+.2f}', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()})
# puntos cerca de la pared, ordenados en y (pose creída)
m = (px > 6.0) & (px < 6.8) & (py > 10.6) & (py < 12.6)
for X, Y in sorted(zip(px[m], py[m]), key=lambda p: p[1]):
    print(f'  y {Y:6.3f}  x {X:6.3f}  {"#"*int(max(0,(X-6.0))*100)}')
