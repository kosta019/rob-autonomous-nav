# Offline: qué daban los rayos hacia un obstáculo justo cuando se borró (RUTA_LIBRE)
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
casos = [  # (hora del RUTA_LIBRE, pose del robot x, y, th, obstáculo x, y)
    ('15:37:39', 5.131, 3.382, 1.80, 4.997, 4.917),
    ('15:37:44', 5.081, 3.592, 1.805, 4.705, 5.179),
    ('15:38:03', 4.831, 4.663, 1.447, 5.01, 5.803),
    ('15:38:18', 4.851, 5.416, 1.762, 5.048, 6.066)]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
pend = list(casos); cortas = 0; n = 0
t_ini = datetime.strptime('2026-11-05 15:36:18', '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
t_fin = datetime.strptime('2026-11-05 15:38:21', '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
while r.has_next():
    _, data, t = r.read_next()
    if t < t_ini: continue
    if t > t_fin: break
    m = deserialize_message(data, LaserScan); n += 1
    rr = np.array(m.ranges)
    cortas += bool(np.any(np.isfinite(rr) & (rr >= m.range_min) & (rr < 0.20)))
    for c in list(pend):
        h, x, y, th, ox, oy = c
        if t >= datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9:
            pend.remove(c)
            lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
            d = math.hypot(ox - lx, oy - ly)
            k = int(round(((math.atan2(oy - ly, ox - lx) - th - m.angle_min) % (2 * math.pi)) / m.angle_increment))
            print(f'{h}: obstáculo ({ox},{oy}) a {d:.2f} m; rayos {[round(float(rr[(k + j) % len(rr)]), 3) for j in (-1, 0, 1)]}')
print(f'escaneos durante la ruta: {n}; con alguna lectura < 0.20 m (ecos del robot): {cortas}')
