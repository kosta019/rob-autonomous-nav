# Offline: puntos de la hoja de la puerta en el mundo, con la pose corregida por ajuste_puerta (dx, dth y dy)
import math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
from autonomous_nav import comprobar_pose as cp
x, y, th = 5.696 + 0.185, 11.017 - 0.282, 1.827 + math.radians(0.33)
for h in sys.argv[2:]:
    t0 = datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    lista = []
    while r.has_next() and len(lista) < 10:
        _, data, t = r.read_next()
        if t < t0: continue
        m = deserialize_message(data, LaserScan)
        a = np.array(m.ranges, dtype=float); a[~np.isfinite(a) | (a < m.range_min) | (a > m.range_max)] = np.nan
        lista.append(a)
    rr = cp.mediana_de_scans(lista); ang = m.angle_min + np.arange(len(rr)) * m.angle_increment
    lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
    ok = ~np.isnan(rr)
    px, py = lx + rr[ok] * np.cos(th + ang[ok]), ly + rr[ok] * np.sin(th + ang[ok])
    sel = (px > 5.2) & (px < 6.45) & (py > 11.85) & (py < 12.35)
    print(f'{h}: {sel.sum()} puntos; X {px[sel].min():.3f}..{px[sel].max():.3f}, Y {py[sel].min():.3f}..{py[sel].max():.3f}, '
          f'Y mediana {np.median(py[sel]):.3f}')
    for X, Y in sorted(zip(px[sel], py[sel]))[::3]:
        print(f'   ({X:.2f}, {Y:.3f})', end='')
    print()
