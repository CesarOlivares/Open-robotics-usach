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
Nodo joy_to_twist: joystick (sensor_msgs/Joy) → velocidad del hexápodo (Twist).

El joystick de la ESP32 publica en /hexapod/joystick el valor CONTINUO de
cada eje en [-1, 1] (pauta P9, micro-ROS). Este nodo lo convierte en una
velocidad proporcional, con tres pasos para que el control sea suave:

  1. Zona muerta con reescalado (deadzone d): un potenciómetro en reposo
     nunca marca 0 exacto; bajo d se toma 0. Sobre d se reescala para que la
     salida parta de 0 sin salto y llegue a 1 en el extremo:
         x' = signo(x) · (|x| − d) / (1 − d)
  2. Curva expo (e ∈ [0, 1]):  y = (1 − e) · x' + e · x'³
     Cerca del centro la pendiente es (1 − e): movimientos finos y suaves.
     En el extremo sigue llegando a 1 (velocidad máxima).
  3. Escala: v = y · v_max. La rampa de aceleración de gait.py hace el resto:
     aunque la palanca salte, la velocidad del robot cambia de forma gradual.

Mapeo (un solo joystick de 2 ejes y un botón):
    eje Y (axes[1])  → avance vx       (arriba = adelante)
    eje X (axes[0])  → giro wz         (derecha = girar a la derecha)
                       o paso lateral vy, según el modo
    botón (buttons[0], apretar la palanca) → cambia giro ↔ paso lateral

Si el joystick deja de publicar, este nodo deja de publicar y gait_node se
detiene solo (cmd_timeout): perder la conexión nunca deja al robot andando.

Parámetros (config/joystick.yaml de hexapod_bringup):
    deadzone, expo, v_max [m/s], w_max [rad/s]
    axis_x_sign, axis_y_sign   ±1: corrigen el sentido según cómo quede
                               montado el módulo (se ajusta en la prueba)
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


def apply_deadzone(x, deadzone):
    """Zona muerta con reescalado: 0 bajo `deadzone`, continua y llega a ±1."""
    x = max(-1.0, min(1.0, x))
    if abs(x) <= deadzone:
        return 0.0
    sign = 1.0 if x > 0 else -1.0
    return sign * (abs(x) - deadzone) / (1.0 - deadzone)


def apply_expo(x, expo):
    """Curva expo: (1 − e)·x + e·x³. Suave al centro, 1 en el extremo."""
    return (1.0 - expo) * x + expo * x ** 3


def shape_axis(x, deadzone, expo):
    """Zona muerta + expo: valor del eje en [-1, 1] → fracción de la velocidad máxima."""
    return apply_expo(apply_deadzone(x, deadzone), expo)


def joy_to_velocity(axes, lateral_mode, p):
    """
    Ejes del joystick → (vx, vy, wz).

    axes: [x, y] crudos en [-1, 1]; p: dict con deadzone, expo, v_max,
    w_max, axis_x_sign, axis_y_sign.
    """
    x = p['axis_x_sign'] * (axes[0] if len(axes) > 0 else 0.0)
    y = p['axis_y_sign'] * (axes[1] if len(axes) > 1 else 0.0)
    fx = shape_axis(x, p['deadzone'], p['expo'])
    fy = shape_axis(y, p['deadzone'], p['expo'])
    vx = fy * p['v_max']
    # x positivo = palanca a la derecha → hacia −y (REP-103: y a la izquierda)
    if lateral_mode:
        return vx, -fx * p['v_max'], 0.0
    return vx, 0.0, -fx * p['w_max']


class JoyToTwist(Node):
    """Suscribe /hexapod/joystick y publica /hexapod/cmd_vel."""

    def __init__(self):
        """Lee parámetros y crea suscripción y publicador."""
        super().__init__('joy_to_twist')
        self.p = {name: self.declare_parameter(name, default).value for name, default in (
            ('deadzone', 0.08), ('expo', 0.5), ('v_max', 0.10), ('w_max', 0.45),
            ('axis_x_sign', 1.0), ('axis_y_sign', 1.0))}
        self.lateral = False
        self.last_button = 0
        self.pub = self.create_publisher(Twist, '/hexapod/cmd_vel', 10)
        self.create_subscription(Joy, '/hexapod/joystick', self.on_joy, 10)
        self.get_logger().info('joystick listo: eje Y avanza, eje X gira '
                               '(botón: cambia a paso lateral)')

    def on_joy(self, msg):
        """Convierte un mensaje del joystick en un Twist."""
        button = msg.buttons[0] if msg.buttons else 0
        if button and not self.last_button:   # flanco: se acaba de apretar
            self.lateral = not self.lateral
            self.get_logger().info('modo: ' + ('paso lateral' if self.lateral else 'giro'))
        self.last_button = button

        vx, vy, wz = joy_to_velocity(list(msg.axes), self.lateral, self.p)
        out = Twist()
        out.linear.x, out.linear.y, out.angular.z = vx, vy, wz
        self.pub.publish(out)


def main(args=None):
    """Punto de entrada: ros2 run hexapod_control joy_to_twist."""
    rclpy.init(args=args)
    node = JoyToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
