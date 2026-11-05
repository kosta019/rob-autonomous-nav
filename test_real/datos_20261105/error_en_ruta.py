# Offline: error de pose a lo largo de la ruta con las paredes. Antes de t_cambio: pared izquierda / saliente (rectas
# y = c, dirección X); después: frente de los bancos (x = 5.86, dirección Y). Error = creído - real.
import csv, math, sys
from datetime import datetime
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import rosbag2_py
bag, log, h0, h1, hc = sys.argv[1:6]
filas = [(float(a['timestamp']), float(a['robot_x']), float(a['robot_y']), float(a['robot_yaw'])) for a in csv.DictReader(open(log))]
ts = np.array([f[0] for f in filas])
t0, t1, tc = [datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() for h in (h0, h1, hc)]
def consenso(u, v, maxpend):
    # recta v = a + b*u con más puntos a < 2 cm, entre las que pasan por dos puntos separados > 0.5 m en u
    mejor = (0, 0.0, 0.0)
    for i in range(0, len(u), 2):
        for j in range(i + 1, len(u), 2):
            if abs(u[j] - u[i]) < 0.5: continue
            b = (v[j] - v[i]) / (u[j] - u[i])
            if abs(b) > maxpend: continue
            a = v[i] - b * u[i]
            n = int(np.sum(np.abs(v - (a + b * u)) < 0.02))
            if n > mejor[0]: mejor = (n, a, b)
    return mejor
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
    ok = np.isfinite(rr) & (rr >= m.range_min) & (rr < 3.0)
    lx, ly = x - 0.032 * math.cos(th), y - 0.032 * math.sin(th)
    px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
    hora = datetime.fromtimestamp(t).strftime('%H:%M:%S')
    if t < tc:
        # pared izquierda (y = 0.04, x < 3.93) o saliente (y = 0.96, x > 3.93): puntos con y < y_robot - 0.3
        sel = (py < y - 0.3) & (np.abs(px - x) < 2.0)
        n, aa, b = consenso(px[sel], py[sel], math.tan(math.radians(10)))
        if n < 15: print(f'{hora} robot ({x:.2f}, {y:.2f}): sin pared clara'); continue
        yw = aa + b * x
        ref = 0.96 if (yw > 0.5) else 0.04
        print(f'{hora} robot ({x:.2f}, {y:.2f}, {math.degrees(th):5.1f}°): pared y={ref} vista en y={yw:.3f} ({n} pts) -> '
              f'error rumbo {math.degrees(math.atan(b)):+.1f}°, error Y {-(yw - ref):+.3f}')
    else:
        sel = (px > x + 0.2) & (px < x + 1.5) & (np.abs(py - y) < 1.2) & (py < 7.2)
        n, aa, b = consenso(py[sel], px[sel], math.tan(math.radians(10)))
        if n < 15: print(f'{hora} robot ({x:.2f}, {y:.2f}): sin pared clara'); continue
        xw = aa + b * y
        print(f'{hora} robot ({x:.2f}, {y:.2f}, {math.degrees(th):5.1f}°): bancos x=5.86 vistos en x={xw:.3f} ({n} pts) -> '
              f'error rumbo {-math.degrees(math.atan(b)):+.1f}°, error X {-(xw - 5.86):+.3f}')
