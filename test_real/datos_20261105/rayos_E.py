# Offline: qué devuelven los rayos que apuntan a E (los que deberían "borrar" el obstáculo) con la ruta libre
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
x, y, th, off = 5.122, 5.114, 1.603, -0.032
lx, ly = x + off*math.cos(th), y + off*math.sin(th)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
t0 = datetime.strptime('2026-11-05 14:58:20', '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
while r.has_next():
    _, data, t = r.read_next()
    if t < t0: continue
    m = deserialize_message(data, LaserScan)
    print(f'range_min {m.range_min} range_max {m.range_max} n {len(m.ranges)}')
    for nombre, (ox, oy) in {'E (5.08,6.12)': (5.082, 6.122), 'obst (5.17,6.11)': (5.17, 6.11), 'obst (4.87,6.38)': (4.87, 6.38), 'obst (5.36,6.93)': (5.36, 6.93)}.items():
        d = math.hypot(ox-lx, oy-ly)
        k = int(round(((math.atan2(oy-ly, ox-lx) - th - m.angle_min) % (2*math.pi)) / m.angle_increment))
        n = len(m.ranges)
        print(f'  {nombre}: a {d:.2f} m, rayos {k-1}..{k+1}:', [round(m.ranges[(k+j) % n], 3) for j in (-1, 0, 1)])
    break
