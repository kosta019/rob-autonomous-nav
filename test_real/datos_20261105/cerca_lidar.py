# Offline: lecturas del LiDAR a menos de 0.40 m (ángulo en grados en el sistema del LiDAR: 0 = delante, 180 = detrás)
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
bag, horas = sys.argv[1], sys.argv[2:]
obj = [datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9 for h in horas]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
hecho = set()
while r.has_next() and len(hecho) < len(obj):
    _, data, t = r.read_next()
    for k, o in enumerate(obj):
        if k in hecho or not (o <= t < o + 3e8): continue
        hecho.add(k)
        m = deserialize_message(data, LaserScan)
        rr = np.array(m.ranges); a = np.degrees(m.angle_min + np.arange(len(rr)) * m.angle_increment)
        sel = np.isfinite(rr) & (rr >= m.range_min) & (rr < 0.40)
        print(horas[k], f'{sel.sum()} lecturas < 0.40 m:', [(int(round(x)), round(float(y), 3)) for x, y in zip(a[sel], rr[sel])][:12])
