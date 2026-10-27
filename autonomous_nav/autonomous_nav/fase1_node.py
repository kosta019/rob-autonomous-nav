# Fase I: seguir la lista de waypoints con parada de emergencia y Mission Log.
# Misma estructura que ref/go_stop.py: una clase Node, publicadores y suscriptores en __init__,
# callbacks y main(). La diferencia: los callbacks solo guardan el último mensaje y el control
# va en un timer a ritmo fijo (así se detecta si un sensor deja de llegar).
import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def normalizar(a):
    # Ángulo en (-pi, pi]
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de_cuaternion(q):
    # Yaw sin tf_transformations (en Jazzy da problemas)
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def limitar(valor, maximo):
    return max(-maximo, min(maximo, valor))


class Fase1Node(Node):
    def __init__(self):
        super().__init__('fase1_node')

        # 1. Parámetros. Sin valores por defecto: todo viene de config/fase1.yaml.
        #    Si falta alguno, el nodo falla al arrancar en vez de usar un valor inventado.
        D = Parameter.Type.DOUBLE
        DA = Parameter.Type.DOUBLE_ARRAY
        SA = Parameter.Type.STRING_ARRAY
        self.pose_inicial = self.declare_parameter('initial_pose', DA).value  # [x, y, theta]
        self.pose_final = self.declare_parameter('final_pose', DA).value
        nombres = self.declare_parameter('waypoint_names', SA).value
        xs = self.declare_parameter('waypoints_x', DA).value
        ys = self.declare_parameter('waypoints_y', DA).value
        obligatorios = self.declare_parameter('mandatory_waypoints', SA).value

        control_rate = self.declare_parameter('control_rate', D).value
        self.tol_waypoint = self.declare_parameter('tol_waypoint', D).value
        self.tol_obligatorio = self.declare_parameter('tol_mandatory', D).value
        self.tol_final_yaw = self.declare_parameter('tol_final_yaw', D).value
        self.max_v = self.declare_parameter('max_linear_speed', D).value
        self.max_w = self.declare_parameter('max_angular_speed', D).value
        self.k_lineal = self.declare_parameter('k_linear', D).value
        self.k_angular = self.declare_parameter('k_angular', D).value
        self.angulo_girar = self.declare_parameter('turn_in_place_angle', D).value
        self.angulo_reorientar = self.declare_parameter('reorient_angle', D).value

        self.stop_distance = self.declare_parameter('stop_distance', D).value
        self.cono = math.radians(self.declare_parameter('stop_half_angle_deg', D).value)
        self.timeout = self.declare_parameter('scan_timeout', D).value
        self.lidar_offset_x = self.declare_parameter('lidar_offset_x', D).value
        log_file = self.declare_parameter('log_file', Parameter.Type.STRING).value
        self.log_period = self.declare_parameter('log_period', D).value

        # 2. Lista de objetivos: (nombre, x, y, tolerancia). Al final, la posición de la pose final.
        if not (len(nombres) == len(xs) == len(ys)):
            raise ValueError('waypoint_names, waypoints_x y waypoints_y deben tener la misma longitud')
        posiciones = [nombres.index(n) for n in obligatorios if n in nombres]
        if len(posiciones) != len(obligatorios) or posiciones != sorted(posiciones):
            self.get_logger().warn(f'Los obligatorios {obligatorios} no están todos, o no en orden, en {nombres}')
        self.objetivos = []
        for nombre, x, y in zip(nombres, xs, ys):
            tol = self.tol_obligatorio if nombre in obligatorios else self.tol_waypoint
            self.objetivos.append((nombre, x, y, tol))
        self.objetivos.append(('final', self.pose_final[0], self.pose_final[1], self.tol_obligatorio))
        self.i_objetivo = 0
        self.estado = 'GIRAR'

        # 3. Últimos mensajes de los sensores y cuándo llegaron (según el reloj del nodo)
        self.odom = None
        self.odom0 = None       # primera lectura de /odom: (x0, y0, theta0)
        self.t_odom = None
        self.scan = None
        self.t_scan = None

        # Banderas para registrar los eventos solo cuando cambian (y no en cada ciclo)
        self.sin_sensores = True
        self.frenado = False
        self.t_ultimo_log = None

        # 4. Publicador y suscriptores (QoS como en la plantilla)
        qos_profile_b = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_b)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_b)

        # 5. Mission Log. Un fichero por ejecución (con la fecha) para no machacar el de otra prueba.
        base, ext = os.path.splitext(log_file)
        ruta_log = f"{base}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{ext}"
        self.fichero = open(ruta_log, 'w', newline='')
        self.csv = csv.writer(self.fichero)
        self.csv.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y', 'robot_yaw',
                           'state', 'event', 'error', 'obstacle_x', 'obstacle_y'])
        self.get_logger().info(f'Mission Log en {os.path.abspath(ruta_log)}')

        # 6. Bucle de control a ritmo fijo
        self.create_timer(1.0 / control_rate, self.control_loop)

    # ---------- Callbacks: solo guardan el último mensaje ----------

    def odom_callback(self, msg):
        if self.odom0 is None:
            p = msg.pose.pose
            self.odom0 = (p.position.x, p.position.y, yaw_de_cuaternion(p.orientation))
        self.odom = msg
        self.t_odom = self.get_clock().now()

    def scan_callback(self, msg):
        self.scan = msg
        self.t_scan = self.get_clock().now()

    # ---------- Pose ----------

    def pose_mundo(self):
        # Pose (x, y, theta) del robot en el sistema del enunciado, o None si aún no hay /odom.
        # Es la única función que sabe de dónde sale la pose: con el SLAM solo cambiará esta.
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        x0, y0, th0 = self.odom0

        # a) Pose relativa al arranque: el desplazamiento desde la primera lectura, girado -theta0
        dx = p.position.x - x0
        dy = p.position.y - y0
        x_rel = math.cos(th0) * dx + math.sin(th0) * dy
        y_rel = -math.sin(th0) * dx + math.cos(th0) * dy
        th_rel = yaw_de_cuaternion(p.orientation) - th0

        # b) Al mundo: girar theta_i y trasladar a (x_i, y_i)
        xi, yi, thi = self.pose_inicial
        x = xi + math.cos(thi) * x_rel - math.sin(thi) * y_rel
        y = yi + math.sin(thi) * x_rel + math.cos(thi) * y_rel
        return x, y, normalizar(thi + th_rel)

    # ---------- Seguridad ----------

    def sensor_caducado(self, t):
        # True si el sensor no ha llegado nunca o lleva más de scan_timeout sin llegar
        if t is None:
            return True
        return (self.get_clock().now() - t).nanoseconds * 1e-9 > self.timeout

    def obstaculo_delante(self, x, y, th):
        # Lectura válida más cercana del cono frontal que esté a menos de stop_distance.
        # Devuelve su posición en el mundo (ox, oy), o None si el cono está libre.
        msg = self.scan
        r_min = None
        a_min = 0.0
        for i, r in enumerate(msg.ranges):
            # 0.0, inf, nan o fuera de [range_min, range_max] = sin dato
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            # Ángulo de cada lectura (no suponemos 360 muestras ni que el índice 0 sea el frente)
            a = normalizar(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= self.cono and r < self.stop_distance and (r_min is None or r < r_min):
                r_min = r
                a_min = a
        if r_min is None:
            return None
        # El LiDAR está desplazado lidar_offset_x sobre el eje del robot
        lx = x + self.lidar_offset_x * math.cos(th)
        ly = y + self.lidar_offset_x * math.sin(th)
        return lx + r_min * math.cos(th + a_min), ly + r_min * math.sin(th + a_min)

    # ---------- Control ----------

    def control_loop(self):
        # 1. Sin datos frescos de algún sensor: parada total
        if self.sensor_caducado(self.t_odom) or self.sensor_caducado(self.t_scan):
            self.publicar_vel(0.0, 0.0)
            if not self.sin_sensores:
                self.sin_sensores = True
                self.get_logger().warn('Sin /odom o /scan recientes: parada total')
                x, y, th = self.pose_mundo()
                self.escribir_log(x, y, th, 'TIMEOUT_SENSOR')
            return
        x, y, th = self.pose_mundo()
        if self.sin_sensores:
            self.sin_sensores = False
            self.get_logger().info('Sensores OK')
            self.escribir_log(x, y, th, 'SENSORES_OK')

        # 2. Lo que querría hacer la máquina de estados
        v, w = self.maquina_estados(x, y, th)

        # 3. Parada de emergencia, por encima de cualquier estado. Solo anula la lineal:
        #    si también bloqueara el giro, mirando a una pared cercana nunca podría girarse para salir.
        obstaculo = self.obstaculo_delante(x, y, th) if v > 0.0 else None
        if obstaculo is not None:
            v = 0.0
            if not self.frenado:
                self.frenado = True
                self.get_logger().warn(f'PARADA DE EMERGENCIA: obstáculo en ({obstaculo[0]:.2f}, {obstaculo[1]:.2f})')
                self.escribir_log(x, y, th, 'PARADA_EMERGENCIA', None, obstaculo[0], obstaculo[1])
        elif self.frenado:
            self.frenado = False
            self.get_logger().info('Cono libre: sigo')
            self.escribir_log(x, y, th, 'FIN_PARADA')

        self.publicar_vel(v, w)

        # 4. Fila periódica del log
        ahora = self.get_clock().now()
        if self.t_ultimo_log is None or (ahora - self.t_ultimo_log).nanoseconds * 1e-9 >= self.log_period:
            self.t_ultimo_log = ahora
            self.escribir_log(x, y, th, '')

    def maquina_estados(self, x, y, th):
        # Devuelve (v, w) según el estado, y cambia de estado y de objetivo cuando toca.
        if self.estado == 'FIN':
            return 0.0, 0.0

        if self.estado == 'ORIENTAR':
            error = normalizar(self.pose_final[2] - th)
            if abs(error) < self.tol_final_yaw:
                self.estado = 'FIN'
                self.get_logger().info('FASE I COMPLETADA')
                self.escribir_log(x, y, th, 'FASE I COMPLETADA', error)
                return 0.0, 0.0
            return 0.0, limitar(self.k_angular * error, self.max_w)

        # LLEGADO: se comprueba antes que nada, esté girando o avanzando
        nombre, ox, oy, tol = self.objetivos[self.i_objetivo]
        dist = math.hypot(ox - x, oy - y)
        if dist < tol:
            self.get_logger().info(f'Waypoint {nombre} alcanzado (error {dist:.3f} m)')
            self.escribir_log(x, y, th, f'WAYPOINT {nombre}', dist)
            self.i_objetivo += 1
            self.estado = 'ORIENTAR' if self.i_objetivo == len(self.objetivos) else 'GIRAR'
            return 0.0, 0.0

        # Error de ángulo hacia el objetivo
        error = normalizar(math.atan2(oy - y, ox - x) - th)
        w = limitar(self.k_angular * error, self.max_w)

        # GIRAR: sobre sí mismo hasta quedar casi alineado (el TB3 no puede ir de lado)
        if self.estado == 'GIRAR':
            if abs(error) >= self.angulo_girar:
                return 0.0, w
            self.estado = 'AVANZAR'

        # AVANZAR: v proporcional a la distancia y w corrige el rumbo.
        # Umbral de salida (30°) mayor que el de entrada (10°): histéresis, para no alternar en cada ciclo.
        if abs(error) > self.angulo_reorientar:
            self.estado = 'GIRAR'
            return 0.0, w
        return min(self.k_lineal * dist, self.max_v), w

    # ---------- Salidas ----------

    def publicar_vel(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # reloj del nodo: vale con use_sim_time
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.publisher.publish(msg)

    def escribir_log(self, x, y, th, evento, error=None, ox=None, oy=None):
        def r3(v):
            return '' if v is None else round(v, 3)
        t = self.get_clock().now().nanoseconds * 1e-9  # en Gazebo, tiempo simulado
        self.csv.writerow([round(t, 3), 'I', r3(x), r3(y), r3(th), self.estado, evento,
                           r3(error), r3(ox), r3(oy)])
        self.fichero.flush()  # que no se pierda nada con un Ctrl+C


def main(args=None):
    # Sin el manejador de Ctrl+C de rclpy: así el contexto de ROS sigue vivo en el finally
    # y la velocidad cero llega de verdad al robot (con el de la plantilla se puede perder).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Fase1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publicar_vel(0.0, 0.0)
        node.fichero.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
