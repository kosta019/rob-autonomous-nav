# Puntos del LiDAR alrededor del hueco de la puerta (pose creída), robot parado 13:44:06-13:44:27, y antes (13:43:58-13:44:04).
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
off = -0.032
ventanas = {'parado': ((5.938, 11.256, 1.472), 1793969046.5, 1793969067.0)}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
(x, y, th), t0, t1 = ventanas['parado']
lx, ly = x + off * math.cos(th), y + off * math.sin(th)
celdas = {}
n = 0
while r.has_next():
    _, data, t = r.read_next()
    if not (t0 < t * 1e-9 < t1): continue
    s = deserialize_message(data, LaserScan); n += 1
    rr = np.array(s.ranges, dtype=float); a = s.angle_min + np.arange(len(rr)) * s.angle_increment
    ok = np.isfinite(rr) & (rr > s.range_min) & (rr < 3.0)
    px, py = lx + rr[ok] * np.cos(th + a[ok]), ly + rr[ok] * np.sin(th + a[ok])
    for X, Y, R in zip(px, py, rr[ok]):
        if 5.50 <= X < 7.10 and 10.00 <= Y < 12.40:
            k = (int((X - 5.50) / 0.05), int((Y - 10.00) / 0.05))
            celdas[k] = celdas.get(k, 0) + 1
print(f'{n} scans. Filas: X de 6.00 a 6.60 (cada 4 cm, hacia la pared y el vestíbulo). Columnas: Y de 10.90 a 12.20.')
print('Número = en cuántos scans hay un punto en esa celda (9 = 9 o más). Robot creído en (5.94, 11.26).')
print('Columnas: Y de 10.00 a 12.40 cada 5 cm (| = 11.27 y 12.02, el hueco del mapa)')
print('       ' + ''.join('|' if j in (25, 40) else ' ' for j in range(48)))
for i in range(32):
    fila = ''.join(('.' if celdas.get((i, j), 0) == 0 else str(min(9, celdas[(i, j)]))) for j in range(48))
    print(f'X {5.50 + i * 0.05:.2f} {fila}')
print('       Y 10.0               10.75              11.5               12.25')
