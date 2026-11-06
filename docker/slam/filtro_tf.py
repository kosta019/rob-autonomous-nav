# Reenvía a /tf el /tf grabado en un bag (remapeado a /tf_bag) quitando map->odom: así un slam_toolbox nuevo puede
# publicar el suyo sin mezclarse con el que se grabó. Y reenvía /scan_bag a /scan restando `retraso_scan` s al sello
# (el LDS sella ~0.1 s tarde: scripts/retraso_scan.py). Herramienta de desarrollo (docker/slam/slam_bag.sh).
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class FiltroTf(Node):
    def __init__(self):
        super().__init__('filtro_tf')
        self.pub = self.create_publisher(TFMessage, '/tf', QoSProfile(depth=100))
        self.create_subscription(TFMessage, '/tf_bag', self.callback, QoSProfile(depth=100))
        self.retraso = Duration(seconds=self.declare_parameter('retraso_scan', 0.0).value)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos)
        self.create_subscription(LaserScan, '/scan_bag', self.callback_scan, qos)

    def callback_scan(self, msg):
        msg.header.stamp = (Time.from_msg(msg.header.stamp) - self.retraso).to_msg()
        self.pub_scan.publish(msg)

    def callback(self, msg):
        msg.transforms = [t for t in msg.transforms if t.header.frame_id != 'map']
        if msg.transforms:
            self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FiltroTf())


if __name__ == '__main__':
    main()
