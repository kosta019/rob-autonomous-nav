# Offline: lecturas < 0.45 m en una ventana de tiempo, agrupadas por ángulo (grados del LiDAR)
import sys, collections
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
bag, h0, h1 = sys.argv[1:4]
t0, t1 = [datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9 for h in (h0, h1)]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
n = 0; c = collections.Counter(); ej = {}
while r.has_next():
    _, data, t = r.read_next()
    if t < t0: continue
    if t > t1: break
    m = deserialize_message(data, LaserScan); n += 1
    rr = np.array(m.ranges); a = np.round(np.degrees(m.angle_min + np.arange(len(rr)) * m.angle_increment)).astype(int) % 360
    for i in np.where(np.isfinite(rr) & (rr >= m.range_min) & (rr < 0.45))[0]:
        c[int(a[i])] += 1; ej.setdefault(int(a[i]), []).append(round(float(rr[i]), 3))
print(f'{h0}-{h1}: {n} scans; ángulos con lecturas < 0.45 m:', [(k, v, sorted(ej[k])[:4]) for k, v in c.most_common(8)])
