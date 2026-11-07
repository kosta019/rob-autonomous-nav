#!/usr/bin/env python3
# Dentro del contenedor de la misión. Hace dos cosas con un nodo de ROS:
#  1. Graba la pose REAL del robot y de las personas según Gazebo (no la /odom: si choca, las ruedas patinan).
#     La toma de /world/laboratorio/dynamic_pose/info (puente a tf2_msgs/TFMessage, sin nombres: primero las personas
#     por orden y después el burger). Salida: real.csv (t_sim, rx, ry, ryaw, p0x, p0y, p1x, p1y...), a 10 Hz simulados.
#  2. Mueve a cada persona (cilindro con VelocityControl) de un extremo a otro de su segmento, con pausas en los
#     extremos. Cierra el lazo con la pose real: velocidad hacia el objetivo, frenando al llegar.
# Uso: nodos_sim.py <escenario.yaml> <real.csv> <sim.yaml>   (con use_sim_time)
import math
import sys

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


class NodosSim(Node):
    def __init__(self, esc, ruta_csv):
        super().__init__('nodos_sim', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.personas = esc['personas']
        self.pubs = [self.create_publisher(Twist, f'/model/persona_{k}/cmd_vel', 10) for k in range(len(self.personas))]
        # Estado de cada persona: objetivo (0 = extremo a, 1 = extremo b), instante hasta el que espera
        self.objetivo = [1] * len(self.personas)
        self.espera = [None] * len(self.personas)
        self.parado = [None] * len(self.personas)
        self.parar_a = esc.get('parar_a', 0.45)
        self.parar_s = esc.get('parar_s', 3.0)
        self.t0 = None
        self.ultimo = -1.0
        self.fichero = open(ruta_csv, 'w')
        self.fichero.write('t,rx,ry,ryaw,' + ','.join(f'p{k}x,p{k}y' for k in range(len(self.personas))) + '\n')
        self.create_subscription(TFMessage, '/world/laboratorio/dynamic_pose/info', self.pose_callback, 10)

    def pose_callback(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        n = len(self.personas)
        if len(msg.transforms) < n + 1:
            return
        if self.t0 is None:
            self.t0 = t
        # Personas: control
        poses = [msg.transforms[k].transform.translation for k in range(n)]
        robot = msg.transforms[n].transform.translation
        for k, (pe, pos) in enumerate(zip(self.personas, poses)):
            self.mover(k, pe, pos, t, robot)
        # Burger: grabación a 10 Hz
        if t - self.ultimo < 0.1:
            return
        self.ultimo = t
        r = msg.transforms[n].transform
        q = r.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        fila = [t, r.translation.x, r.translation.y, yaw]
        for p in poses:
            fila += [p.x, p.y]
        self.fichero.write(','.join(f'{v:.3f}' for v in fila) + '\n')
        self.fichero.flush()

    def mover(self, k, pe, pos, t, robot):
        cmd = Twist()
        cmd.linear.z = 2.0 * (pe['alto'] / 2 - pos.z)         # sin gravedad: se mantiene a su altura
        if t - self.t0 >= pe['arranque']:
            if self.espera[k] is not None and t < self.espera[k]:
                pass                                          # pausa en el extremo
            else:
                self.espera[k] = None
                ox, oy = pe['extremo_b'] if self.objetivo[k] == 1 else pe['extremo_a']
                d = math.hypot(ox - pos.x, oy - pos.y)
                # Persona cortés: si el robot está delante, muy cerca, se para (si no, lo empujaría como un
                # cuerpo de masa infinita). Pasados parar_s s sin que se aparte, da media vuelta.
                dr = math.hypot(robot.x - pos.x, robot.y - pos.y)
                delante = d > 0 and ((robot.x - pos.x) * (ox - pos.x) + (robot.y - pos.y) * (oy - pos.y)) > 0
                if self.parar_a > 0 and dr < self.parar_a and delante:
                    if self.parado[k] is None:
                        self.parado[k] = t
                    if t - self.parado[k] > self.parar_s:
                        self.objetivo[k] = 1 - self.objetivo[k]
                        self.parado[k] = None
                    return self.pubs[k].publish(cmd)          # parada: velocidad cero
                self.parado[k] = None
                if d < 0.05:
                    self.objetivo[k] = 1 - self.objetivo[k]
                    self.espera[k] = t + pe['pausa']
                else:
                    v = min(pe['velocidad'], d / 0.2)          # frena al llegar
                    cmd.linear.x = v * (ox - pos.x) / d
                    cmd.linear.y = v * (oy - pos.y) / d
        self.pubs[k].publish(cmd)


def main():
    with open(sys.argv[1]) as f:
        esc = yaml.safe_load(f)
    with open(sys.argv[3]) as f:
        esc.update({k: v for k, v in yaml.safe_load(f)['persona'].items() if k in ('parar_a', 'parar_s')})
    rclpy.init()
    nodo = NodosSim(esc, sys.argv[2])
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        nodo.fichero.close()


if __name__ == '__main__':
    main()
