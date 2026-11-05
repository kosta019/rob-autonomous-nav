# Herramienta de prueba: gira el robot sobre sí mismo N vueltas según el giróscopo y compara
# el giro medido por /imu con el de /odom. El resto (con signo, en (-180, 180]) es lo que hay que comparar
# con el giro real medido con las paredes.
# Misma estructura que ref/go_stop.py: una clase Node, pubs/subs en __init__, callbacks y main().
import csv
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def normalizar(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PruebaRumbo(Node):
    def __init__(self):
        super().__init__('prueba_rumbo')
        # Parámetros (con valor por defecto: es una herramienta de prueba)
        self.w_max = self.declare_parameter('velocidad_angular', 0.5).value  # rad/s
        self.vueltas = self.declare_parameter('vueltas', 4.0).value
        self.objetivo = self.vueltas * 2.0 * math.pi

        self.giro_imu = 0.0      # giro acumulado del giróscopo (sin envolver)
        self.giro_odom = 0.0     # giro acumulado de /odom (desenrollado)
        self.t_imu_anterior = None
        self.th_odom_anterior = None
        self.hay_imu = False
        self.hay_odom = False
        self.terminado = False

        qos_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_r)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_b)
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_b)

        self.fichero = open(f"prueba_rumbo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", 'w', newline='')
        self.csv = csv.writer(self.fichero)
        self.csv.writerow(['t', 'giro_odom', 'giro_imu'])
        self.create_timer(0.1, self.control_loop)  # 10 Hz

    def odom_callback(self, msg):
        th = yaw_de_cuaternion(msg.pose.pose.orientation)
        if self.th_odom_anterior is not None:
            self.giro_odom += normalizar(th - self.th_odom_anterior)
        self.th_odom_anterior = th
        self.hay_odom = True

    def imu_callback(self, msg):
        # Integra la velocidad angular con el dt entre los sellos de tiempo del /imu
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t_imu_anterior is not None and 0.0 < t - self.t_imu_anterior < 0.5:
            self.giro_imu += msg.angular_velocity.z * (t - self.t_imu_anterior)
        self.t_imu_anterior = t
        self.hay_imu = True

    def control_loop(self):
        if self.terminado or not (self.hay_imu and self.hay_odom):
            return
        restante = self.objetivo - self.giro_imu
        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv.writerow([round(t, 3), round(self.giro_odom, 4), round(self.giro_imu, 4)])
        if restante <= 0.0:
            self.terminado = True
            self.publicar_vel(0.0)
            return
        # Frena al final: proporcional a lo que falta, con mínimo para que no se pare antes
        w = max(0.05, min(self.w_max, 1.5 * restante))
        self.publicar_vel(w)

    def publicar_vel(self, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.angular.z = w
        self.publisher.publish(msg)

    def resumen(self):
        for nombre, giro in (('/odom', self.giro_odom), ('/imu', self.giro_imu)):
            grados = math.degrees(giro)
            print(f'Giro según {nombre}: {grados:.1f} grados; resto con signo: {math.degrees(normalizar(giro)):.2f} grados')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PruebaRumbo()
    try:
        while rclpy.ok() and not node.terminado:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Unos segundos más para que el robot se pare y /odom e /imu acaben de llegar
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.publicar_vel(0.0)
        node.fichero.close()
        node.resumen()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
