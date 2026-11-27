# Copyright 2026 Cesar Olivares Lavalovich
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Nodo de postura y marcha del hexápodo.

Al partir lleva el robot de forma suave a la postura de pie:
    1. RECOGER  desde los ángulos medidos (/joint_states) hasta las patas
                recogidas, interpolando cada articulación (tuck_time s).
    2. PARAR    los pies bajan en línea recta (IK en cada paso) desde la
                postura recogida hasta la de pie: el cuerpo sube (stand_time s).
    3. DE PIE   mantiene la postura y camina con la marcha trípode (gait.py)
                según la velocidad pedida en /hexapod/cmd_vel.
Si se pidiera la postura de pie de golpe, los servos saltarían a su
velocidad máxima y el robot rebotaría.

Las interpolaciones usan s(t) = (1 − cos πt)/2: parte y llega con velocidad 0.

Tópicos:
    sub  /joint_states                 ángulos medidos (solo para partir)
    sub  /hexapod/cmd_vel              geometry_msgs/Twist: linear.x, linear.y [m/s]
                                       y angular.z [rad/s]. Si deja de llegar por
                                       cmd_timeout s, se toma como velocidad 0.
    pub  /legs_controller/commands     std_msgs/Float64MultiArray, 18 ángulos
                                       [rad] en el orden de HexapodModel.joint_names()

Parámetros (config/gait.yaml de hexapod_bringup):
    robot_description   URDF en texto: de ahí salen montaje y largos de las patas
    rate                frecuencia de publicación [Hz]
    stand_radial        pie de pie: distancia horizontal eje del FÉMUR → pie [m]
    stand_height        pie de pie: altura del eje de la coxa sobre el pie [m]
    tuck_radial, tuck_height   lo mismo, con las patas recogidas
    tuck_time, stand_time      duración de cada fase [s]
    period, duty, step_height, max_step, max_accel, max_alpha   marcha (gait.py)
    cmd_timeout         [s] sin comandos → se detiene
"""

import math

from geometry_msgs.msg import Twist
from hexapod_control.gait import GaitParams, TripodGait
from hexapod_control.kinematics import UnreachableError
from hexapod_control.robot_model import HexapodModel, LEG_NAMES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def smooth(t):
    """Perfil (1 − cos πt)/2 para t en [0, 1]: velocidad 0 al inicio y al final."""
    t = min(max(t, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * t)


def lerp(a, b, s):
    """Interpolación lineal elemento a elemento entre las secuencias a y b."""
    return [x + (y - x) * s for x, y in zip(a, b)]


class GaitNode(Node):
    """Lleva el robot a la postura de pie y lo hace caminar."""

    def __init__(self):
        """Lee parámetros y modelo, y crea el publicador y el temporizador."""
        super().__init__('gait_node')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('robot_description', ''), ('rate', 50.0),
            ('stand_radial', 0.060), ('stand_height', 0.100),
            ('tuck_radial', 0.080), ('tuck_height', 0.030),
            ('tuck_time', 1.0), ('stand_time', 2.5),
            ('period', 1.0), ('duty', 0.5), ('step_height', 0.03), ('max_step', 0.06),
            ('max_accel', 0.4), ('max_alpha', 2.0), ('cmd_timeout', 0.5))}
        if not p['robot_description']:
            raise RuntimeError('falta el parámetro robot_description')
        self.p = p
        self.model = HexapodModel(p['robot_description'])
        self.names = self.model.joint_names()

        # Las distancias "radial" se miden desde el eje del FÉMUR (como en el
        # plan): al eje de la coxa se le suma L1.
        l1 = self.model.legs[0].geom.coxa
        self.tuck_feet = self.model.neutral_feet(l1 + p['tuck_radial'], p['tuck_height'])
        self.stand_feet = self.model.neutral_feet(l1 + p['stand_radial'], p['stand_height'])
        self.tuck_angles = self.model.joint_angles(self.tuck_feet)
        self.stand_angles = self.model.joint_angles(self.stand_feet)

        self.gait = TripodGait(LEG_NAMES, self.stand_feet, GaitParams(
            period=p['period'], duty=p['duty'], step_height=p['step_height'],
            max_step=p['max_step'], max_accel=p['max_accel'], max_alpha=p['max_alpha']))
        self.cmd = ((0.0, 0.0), 0.0)
        self.cmd_time = None
        self.last_t = None
        self.last_angles = list(self.stand_angles)

        self.start_angles = None
        self.t0 = None
        self.phase = 'esperando /joint_states'

        self.pub = self.create_publisher(Float64MultiArray, '/legs_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(Twist, '/hexapod/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / p['rate'], self.step)
        self.get_logger().info(
            f'de pie: pie a {p["stand_radial"] * 1000:.0f} mm del fémur y '
            f'{p["stand_height"] * 1000:.0f} mm bajo la coxa')

    def on_joint_states(self, msg):
        """Guarda los ángulos medidos la primera vez: son el punto de partida."""
        # Sin /clock todavía el tiempo simulado vale 0: se espera a que llegue,
        # si no la primera fase se saltaría.
        if self.start_angles is not None or self.now() == 0.0:
            return
        pos = dict(zip(msg.name, msg.position))
        if not all(n in pos for n in self.names):
            return
        self.start_angles = [pos[n] for n in self.names]
        self.t0 = self.now()

    def on_cmd_vel(self, msg):
        """Guarda la velocidad pedida y cuándo llegó."""
        self.cmd = ((msg.linear.x, msg.linear.y), msg.angular.z)
        self.cmd_time = self.now()

    def current_cmd(self):
        """Velocidad pedida, o 0 si el último comando es más viejo que cmd_timeout."""
        if self.cmd_time is None or self.now() - self.cmd_time > self.p['cmd_timeout']:
            return (0.0, 0.0), 0.0
        return self.cmd

    def walk(self):
        """Un paso de la marcha: pies según la velocidad, y la IK de las 6 patas."""
        now = self.now()
        dt = 0.0 if self.last_t is None else min(max(now - self.last_t, 0.0), 0.1)
        self.last_t = now
        v, wz = self.current_cmd()
        feet = self.gait.update(dt, v, wz)
        try:
            self.last_angles = self.model.joint_angles(feet)
        except UnreachableError as e:
            # No debería pasar (max_step lo evita): se mantienen los ángulos
            # anteriores en vez de mandar uno imposible.
            self.get_logger().warn(f'pie fuera de alcance, se mantiene la pose: {e}',
                                   throttle_duration_sec=1.0)
        return self.last_angles

    def now(self):
        """Tiempo actual en segundos (simulado si use_sim_time es true)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def set_phase(self, phase):
        """Registra el cambio de fase en el log."""
        if phase != self.phase:
            self.phase = phase
            self.get_logger().info(f'fase: {phase}')

    def step(self):
        """Calcula y publica los 18 ángulos de este instante."""
        if self.start_angles is None:
            return
        t = self.now() - self.t0
        tuck_t, stand_t = self.p['tuck_time'], self.p['stand_time']
        if t < tuck_t:
            self.set_phase('recoger')
            angles = lerp(self.start_angles, self.tuck_angles, smooth(t / tuck_t))
        elif t < tuck_t + stand_t:
            self.set_phase('parar')
            s = smooth((t - tuck_t) / stand_t)
            feet = [lerp(a, b, s) for a, b in zip(self.tuck_feet, self.stand_feet)]
            angles = self.model.joint_angles(feet)
        else:
            self.set_phase('de pie / marcha')
            angles = self.walk()
        self.pub.publish(Float64MultiArray(data=list(angles)))


def main(args=None):
    """Punto de entrada del nodo."""
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
