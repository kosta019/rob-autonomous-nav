# Offline: la relocalización tal como la hace fase1_node, con 10 scans del bag con el robot parado delante de la puerta
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
from autonomous_nav import comprobar_pose as cp
from autonomous_nav.fase1_node import decidir_reloc
creida = (5.696, 11.017, 1.827)
t0 = datetime.strptime('2026-11-05 ' + sys.argv[2], '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
lista = []
while r.has_next() and len(lista) < 10:
    _, data, t = r.read_next()
    if t < t0: continue
    m = deserialize_message(data, LaserScan)
    a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
    lista.append(a); info = (m.angle_min, m.angle_increment, m.range_min, m.range_max)
x, y, th, res = cp.estimar(creida, -0.032, cp.mediana_de_scans(lista), info, cp.cargar_caras(), '')
ok, motivo = decidir_reloc(creida, (x, y, th), res, 0.30, math.radians(3.0), 0.02, 30)
print(f'{sys.argv[2]}: nueva ({x:.3f}, {y:.3f}, {math.degrees(th):.2f}°)  dx={x-creida[0]:+.3f} dy={y-creida[1]:+.3f} dth={math.degrees(cp.normalizar(th-creida[2])):+.2f}°  -> {"ACEPTADA" if ok else "RECHAZADA " + motivo}')
print('   caras:', [(c['eje'], round(c['c'], 2), c['n'], round(c['rms'] * 100, 2)) for c in res])
