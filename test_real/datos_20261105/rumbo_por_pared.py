# Offline: lee /scan del bag con el robot quieto en A y calcula el rumbo del LiDAR que da cada pared por separado.
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
from autonomous_nav import comprobar_pose as cp

bag, t0s, t1s = sys.argv[1], sys.argv[2], sys.argv[3]
dia = '2026-11-05 '
t0 = datetime.strptime(dia + t0s, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
t1 = datetime.strptime(dia + t1s, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
scans, info = [], None
while r.has_next() and len(scans) < cp.N_SCANS:
    _, data, t = r.read_next()
    if t < t0 or t > t1:
        continue
    m = deserialize_message(data, LaserScan)
    info = (m.angle_min, m.angle_increment, m.range_min, m.range_max)
    a = np.array(m.ranges, dtype=float)
    a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    scans.append(a)
print(f'{len(scans)} scans entre {t0s} y {t1s}')
med = cp.mediana_de_scans(scans)
caras = cp.cargar_caras()
x, y, th, resumen = cp.estimar([0.70, 0.92, 0.0], -0.032, med, info, caras)
print(f'Global: x {x:.4f}  y {y:.4f}  rumbo LiDAR {math.degrees(th):.3f} gr')
ang = info[0] + np.arange(len(med)) * info[1]
rr = np.where(np.isnan(med), 0.0, med)
p = np.stack([rr * np.cos(ang), rr * np.sin(ang)], axis=1)
for c in resumen:
    pts = p[c['sel']]
    _, vec = np.linalg.eigh(np.cov(pts.T))
    phi = math.atan2(vec[1, 1], vec[0, 1])
    direccion = math.pi / 2 if c['eje'] == 'X' else 0.0
    d = cp.normalizar(direccion - phi - th)
    d = d - math.pi * round(d / math.pi)
    print(f"  pared cara {c['eje']} c={c['c']:.3f}: {c['n']} puntos -> rumbo LiDAR {math.degrees(th + d):.3f} gr (dif {math.degrees(d):+.3f})")
