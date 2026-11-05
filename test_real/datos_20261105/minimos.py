import sys, collections
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
t0 = datetime.strptime('2026-11-05 15:11:45', '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
n = 0; cortas = collections.Counter(); mins = []
while r.has_next():
    _, data, t = r.read_next()
    if t < t0: continue
    m = deserialize_message(data, LaserScan); n += 1
    rr = np.array(m.ranges); ok = np.isfinite(rr) & (rr >= m.range_min)
    mins.append(rr[ok].min())
    for i in np.where(ok & (rr < 0.45))[0]: cortas[int(round(np.degrees(m.angle_min + i*m.angle_increment)))] += 1
print(n, 'scans; mínimo de cada scan: min', round(min(mins), 3), 'mediana', round(float(np.median(mins)), 3))
print('ángulos con lecturas < 0.45 m (grados: veces):', sorted(cortas.items())[:30])
print('range_min', m.range_min, 'lecturas entre 0 y range_min:', int(np.sum((rr > 0) & (rr < m.range_min))))
