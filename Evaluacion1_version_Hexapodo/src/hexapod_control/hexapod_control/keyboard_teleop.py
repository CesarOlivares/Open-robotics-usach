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
Nodo keyboard_teleop: maneja el hexápodo con el teclado (adaptado de Octavius).

Cada tecla sube o baja una componente de la velocidad; el nodo publica la
velocidad actual a 10 Hz en /hexapod/cmd_vel (geometry_msgs/Twist), que
escucha gait_node. Se publica aunque no se toque nada porque gait_node se
detiene si deja de recibir comandos (cmd_timeout).

Por qué Twist: es el mensaje estándar de ROS para pedir velocidad a una base
móvil. El joystick de la ESP32 (E6) termina en el mismo tópico.

Uso (en una terminal propia, porque necesita leer el teclado):
    ros2 run hexapod_control keyboard_teleop
"""

import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

V_STEP = 0.02     # [m/s] por pulsación
W_STEP = 0.1      # [rad/s] por pulsación
V_MAX = 0.10      # [m/s]  = max_step / (β T) con los valores de gait.yaml
W_MAX = 0.5       # [rad/s]

# tecla → (componente, signo); componentes: 0 = vx, 1 = vy, 2 = wz
KEYS = {
    'w': (0, 1.0), 's': (0, -1.0),
    'a': (1, 1.0), 'd': (1, -1.0),
    'q': (2, 1.0), 'e': (2, -1.0),
}

HELP = """
Hexápodo — control por teclado
------------------------------
  w / s : adelante / atrás          (±0,02 m/s por pulsación)
  a / d : izquierda / derecha       (±0,02 m/s)
  q / e : girar izquierda / derecha (±0,1 rad/s)
  espacio: detener
  Ctrl+C : salir
"""


def apply_key(cmd, key):
    """
    Nueva velocidad (vx, vy, wz) tras una tecla, con los límites aplicados.

    Devuelve None si la tecla no tiene función.
    """
    key = key.lower()
    if key == ' ':
        return (0.0, 0.0, 0.0)
    if key not in KEYS:
        return None
    index, sign = KEYS[key]
    step, top = (W_STEP, W_MAX) if index == 2 else (V_STEP, V_MAX)
    new = list(cmd)
    new[index] = round(max(-top, min(top, new[index] + sign * step)), 6)
    return tuple(new)


class KeyboardTeleop(Node):
    """Guarda la velocidad elegida y la publica a 10 Hz."""

    def __init__(self):
        """Crea el publicador y el temporizador."""
        super().__init__('keyboard_teleop')
        self.cmd = (0.0, 0.0, 0.0)
        self.pub = self.create_publisher(Twist, '/hexapod/cmd_vel', 10)
        self.create_timer(0.1, self.publish)

    def on_key(self, key):
        """Actualiza la velocidad con una tecla y la muestra."""
        new = apply_key(self.cmd, key)
        if new is not None:
            self.cmd = new
            print(f'\rvx = {new[0]:+.2f} m/s   vy = {new[1]:+.2f} m/s   '
                  f'wz = {new[2]:+.2f} rad/s   ', end='', flush=True)

    def publish(self):
        """Publica la velocidad actual."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.cmd
        self.pub.publish(msg)


def read_key(timeout):
    """Espera una tecla hasta timeout segundos; devuelve '' si no hubo."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None):
    """Punto de entrada: ros2 run hexapod_control keyboard_teleop."""
    if not sys.stdin.isatty():
        print('keyboard_teleop necesita una terminal interactiva: '
              'ejecútalo con "ros2 run" en una terminal propia.')
        return
    rclpy.init(args=args)
    node = KeyboardTeleop()
    original = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        # cbreak: cada tecla llega al instante, sin Enter. Se restaura al salir.
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.05)
            if key:
                node.on_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
