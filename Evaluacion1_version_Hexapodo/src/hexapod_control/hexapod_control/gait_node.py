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
    3. DE PIE   mantiene la postura.
Si se pidiera la postura de pie de golpe, los servos saltarían a su
velocidad máxima y el robot rebotaría.

Las interpolaciones usan s(t) = (1 − cos πt)/2: parte y llega con velocidad 0.

Tópicos:
    sub  /joint_states                 ángulos medidos (solo para partir)
    pub  /legs_controller/commands     std_msgs/Float64MultiArray, 18 ángulos
                                       [rad] en el orden de HexapodModel.joint_names()

Parámetros (config/gait.yaml de hexapod_bringup):
    robot_description   URDF en texto: de ahí salen montaje y largos de las patas
    rate                frecuencia de publicación [Hz]
    stand_radial        pie de pie: distancia horizontal eje de coxa → pie [m]
    stand_height        pie de pie: altura del eje de la coxa sobre el pie [m]
    tuck_radial, tuck_height   lo mismo, con las patas recogidas
    tuck_time, stand_time      duración de cada fase [s]
"""

import math

from hexapod_control.robot_model import HexapodModel
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
    """Lleva el robot a la postura de pie y la mantiene."""

    def __init__(self):
        """Lee parámetros y modelo, y crea el publicador y el temporizador."""
        super().__init__('gait_node')
        p = {name: self.declare_parameter(name, default).value for name, default in (
            ('robot_description', ''), ('rate', 50.0),
            ('stand_radial', 0.070), ('stand_height', 0.090),
            ('tuck_radial', 0.080), ('tuck_height', 0.030),
            ('tuck_time', 1.0), ('stand_time', 2.5))}
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

        self.start_angles = None
        self.t0 = None
        self.phase = 'esperando /joint_states'

        self.pub = self.create_publisher(Float64MultiArray, '/legs_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
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
            self.set_phase('de pie')
            angles = self.stand_angles
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
