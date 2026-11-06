# /odom, /scan e /imu alrededor del salto de pose de las 13:58:18 (1793969898).
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
t0, t1 = 1793969893.0, 1793969905.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/odom', '/scan', '/imu']))
ultimo = {}
cuenta = {}
odoms = []
while r.has_next():
    topic, data, t = r.read_next()
    t *= 1e-9
    if not (t0 < t < t1): continue
    if topic in ultimo and t - ultimo[topic] > 0.4:
        print(f'{t - 1793969880:7.2f}  hueco en {topic}: {t - ultimo[topic]:.2f} s sin mensajes')
    ultimo[topic] = t
    cuenta[topic] = cuenta.get(topic, 0) + 1
    if topic == '/odom':
        o = deserialize_message(data, Odometry)
        p, q = o.pose.pose.position, o.pose.pose.orientation
        odoms.append((t, p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))))
print('mensajes por topic:', cuenta, '| último de cada uno (s desde 13:58:00):', {k: round(v - 1793969880, 2) for k, v in ultimo.items()})
prev = None
for t, x, y, yaw in odoms:
    if prev is None or math.hypot(x - prev[1], y - prev[2]) > 0.05 or t - prev[0] > 0.3:
        print(f'{t - 1793969880:7.2f}  odom ({x:.3f}, {y:.3f}, {yaw:.3f})' + ('' if prev is None else f'  salto {math.hypot(x - prev[1], y - prev[2]):.3f} m'))
    prev = (t, x, y, yaw)
print(f'última odom: ({odoms[-1][1]:.3f}, {odoms[-1][2]:.3f}) a los {odoms[-1][0] - 1793969880:.2f} s')
