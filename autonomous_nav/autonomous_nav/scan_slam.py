# Reenvía /scan a /scan_slam con el sello de tiempo corregido, para slam_toolbox (el mapa entregable).
# El LDS sella cada vuelta al acabarla: el scan llega ~0.1 s tarde respecto a la pose (scripts/retraso_scan.py, 06/11).
# Girando a 0.9 rad/s eso son ~5°, y el SLAM tuerce el mapa (análisis en docs/analisis_slam_20261106.md). fase1_node no lo
# usa: sigue con /scan (no corrige con scans tomados girando deprisa).
# Misma estructura que ref/go_stop.py: una clase Node, publicador y suscriptor en __init__, callback y main().
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def resellar(stamp, offset):
    # Sello (builtin_interfaces/Time) desplazado 'offset' segundos (negativo = antes)
    return (Time.from_msg(stamp) + Duration(nanoseconds=int(round(offset * 1e9)))).to_msg()


class ScanSlam(Node):
    def __init__(self):
        super().__init__('scan_slam')
        self.offset = self.declare_parameter('stamp_offset', -0.10).value   # s: medido el 06/11 (retraso_scan.py)
        # QoS de /scan como en la plantilla (BEST_EFFORT); slam_toolbox se suscribe también en BEST_EFFORT
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.publisher = self.create_publisher(LaserScan, '/scan_slam', qos_profile_b)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)
        self.get_logger().info(f'/scan -> /scan_slam con el sello {self.offset:+.3f} s')

    def scan_callback(self, msg):
        msg.header.stamp = resellar(msg.header.stamp, self.offset)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
