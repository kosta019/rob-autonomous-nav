# Offline: error de rumbo a lo largo de la ruta = pendiente de la pared de la derecha (en dirección Y en el mundo:
# frente de los bancos X 5.86 para Y < 7.30, pared de la sala X 6.35 después) en el sistema creído del robot.
import csv, math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
bag, log, h0, h1 = sys.argv[1:5]
filas = [(float(a['timestamp']), float(a['robot_x']), float(a['robot_y']), float(a['robot_yaw'])) for a in csv.DictReader(open(log))]
ts = np.array([f[0] for f in filas])
t0, t1 = [datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() for h in (h0, h1)]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
ultimo = 0
while r.has_next():
    _, data, t = r.read_next()
    t = t / 1e9
    if t < t0: continue
    if t > t1: break
    if t - ultimo < 2.0: continue
    ultimo = t
    _, x, y, th = filas[int(np.argmin(np.abs(ts - t)))]
    m = deserialize_message(data, LaserScan)
    rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
    ok = np.isfinite(rr) & (rr >= m.range_min) & (rr < 2.5)
    lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
    px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
    # puntos a la derecha del robot (X mayor), en una franja de Y alrededor de él
    sel = (px > x + 0.2) & (px < x + 1.5) & (np.abs(py - y) < 1.2)
    qx, qy = px[sel], py[sel]
    mejor = (0, 0.0, 0.0)
    for i in range(len(qy)):
        for j in range(i + 1, len(qy)):
            if abs(qy[j] - qy[i]) < 0.3: continue
            b = (qx[j] - qx[i]) / (qy[j] - qy[i])
            if abs(b) > math.tan(math.radians(15)): continue
            aa = qx[i] - b * qy[i]
            n = int(np.sum(np.abs(qx - (aa + b * qy)) < 0.02))
            if n > mejor[0]: mejor = (n, aa, b)
    n, aa, b = mejor
    hora = datetime.fromtimestamp(t).strftime('%H:%M:%S')
    if n >= 12:
        print(f'{hora} robot ({x:.2f}, {y:.2f}, {math.degrees(th):5.1f}°): pared derecha {n:2d} pts en X {aa + b * y:.2f} '
              f'-> error de rumbo {-math.degrees(math.atan(b)):+.1f}°')
    else:
        print(f'{hora} robot ({x:.2f}, {y:.2f}): sin pared clara ({n} pts)')
