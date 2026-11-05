# Offline: /cmd_vel del bag en una ventana: cuántos mensajes, valores de v y w, y si hay mensajes muy seguidos (dos fuentes)
import sys, collections
from datetime import datetime
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TwistStamped
import rosbag2_py
t0, t1 = [datetime.strptime('2026-11-05 ' + h, '%Y-%m-%d %H:%M:%S').timestamp() * 1e9 for h in sys.argv[2:4]]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/cmd_vel']))
msgs = []
while r.has_next():
    _, data, t = r.read_next()
    if t0 <= t <= t1:
        m = deserialize_message(data, TwistStamped)
        msgs.append((t, m.twist.linear.x, m.twist.angular.z, m.header.frame_id, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
print(len(msgs), 'mensajes de /cmd_vel entre', sys.argv[2], 'y', sys.argv[3], f'({len(msgs) / ((t1 - t0) / 1e9):.1f} por segundo)')
vs = [v for _, v, _, _, _ in msgs]
print('v: max', max(vs) if vs else '-', ' valores distintos más frecuentes:', collections.Counter(round(v, 3) for v in vs).most_common(6))
print('frame_id:', collections.Counter(f for *_, f, _ in msgs).most_common(3))
for t, v, w, f, st in msgs[:: max(1, len(msgs) // 15)]:
    print(f"  {datetime.fromtimestamp(t / 1e9).strftime('%H:%M:%S.%f')[:12]}  v {v:.3f}  w {w:+.3f}")
