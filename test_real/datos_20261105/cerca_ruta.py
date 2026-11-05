# Offline: con el robot parado en ESPERAR, ¿qué ve el LiDAR cerca de la ruta pendiente? (para ver si es estático)
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
bag = sys.argv[1]
x, y, th, off = 5.122, 5.114, 1.603, -0.032          # pose creída en ESPERAR (Mission Log)
ruta = [(x, y), (5.082, 6.122), (5.965, 11.655)]      # robot -> E -> E-Door_1
lx, ly = x + off * math.cos(th), y + off * math.sin(th)
def dist_ruta(px, py):
    d = np.full(px.shape, np.inf)
    for (ax, ay), (bx, by) in zip(ruta, ruta[1:]):
        vx, vy = bx - ax, by - ay
        t = np.clip(((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy), 0, 1)
        d = np.minimum(d, np.hypot(px - (ax + t * vx), py - (ay + t * vy)))
    return d
ventanas = sys.argv[2:]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
obj = [datetime.strptime('2026-11-05 ' + v, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9 for v in ventanas]
hecho = set()
while r.has_next() and len(hecho) < len(obj):
    _, data, t = r.read_next()
    for k, o in enumerate(obj):
        if k in hecho or not (o <= t < o + 3e8):
            continue
        hecho.add(k)
        m = deserialize_message(data, LaserScan)
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        rr = np.array(m.ranges)
        ok = np.isfinite(rr) & (rr >= m.range_min) & (rr <= 2.5)
        px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
        d = dist_ruta(px, py)
        sel = d < 0.40
        pts = sorted(zip(np.round(px[sel], 2), np.round(py[sel], 2), np.round(d[sel], 2)), key=lambda q: q[1])
        print(f'{ventanas[k]}: {sel.sum()} puntos a < 0.40 m de la ruta:', pts[:14])
