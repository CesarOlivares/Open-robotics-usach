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
Nodo keyboard_teleop: lee el teclado y publica cada pulsación en ROS.

No decide ángulos ni conoce límites: solo traduce teclas a un mensaje
sensor_msgs/Joy en /octavius/keyboard. arm_commander lo convierte en
movimiento. Así la lógica de control queda en un solo lugar.

Por qué sensor_msgs/Joy:
    Es el mensaje estándar de ROS para mandos (ejes + botones). El joystick
    de la ESP32 usa el mismo tipo, así ambos se procesan igual.

Mensaje publicado por cada tecla:
    axes[0]  codo          w = +1, s = -1
    axes[1]  muñeca pitch  r = +1, f = -1
    axes[2]  muñeca roll   e = +1, q = -1
    buttons[0] = 1         a: abrir / cerrar garra
    buttons[1] = 1         h: volver a home

Mantener una tecla presionada repite la pulsación (lo hace la terminal),
así que el brazo avanza un paso por cada repetición.

Uso (en una terminal propia, porque necesita leer el teclado):
    ros2 run octavius_bringup keyboard_teleop
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# tecla → (índice de eje, dirección)
AXIS_KEYS = {
    'w': (0, 1.0), 's': (0, -1.0),
    'r': (1, 1.0), 'f': (1, -1.0),
    'e': (2, 1.0), 'q': (2, -1.0),
}
# tecla → índice de botón
BUTTON_KEYS = {'a': 0, 'h': 1}

NUM_AXES = 3
NUM_BUTTONS = 2

HELP = """
Octavius — control por teclado
------------------------------
  w / s : codo           arriba / abajo
  r / f : muñeca pitch   arriba / abajo
  q / e : girar garra    izquierda / derecha
  a     : abrir / cerrar garra
  h     : volver a home
  Ctrl+C: salir

(La base y el hombro se mueven con el joystick de la ESP32.)
"""


def key_to_joy(key: str):
    """
    Traduce una tecla a los ejes y botones de un mensaje Joy.

    Devuelve (axes, buttons), o None si la tecla no tiene función.
    """
    key = key.lower()
    axes = [0.0] * NUM_AXES
    buttons = [0] * NUM_BUTTONS
    if key in AXIS_KEYS:
        index, direction = AXIS_KEYS[key]
        axes[index] = direction
    elif key in BUTTON_KEYS:
        buttons[BUTTON_KEYS[key]] = 1
    else:
        return None
    return axes, buttons


class KeyboardTeleop(Node):
    """Publica un sensor_msgs/Joy por cada tecla válida."""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(Joy, '/octavius/keyboard', 10)

    def publish_key(self, key: str) -> None:
        """Publica la tecla si tiene una función asignada."""
        result = key_to_joy(key)
        if result is None:
            return
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes, msg.buttons = result
        self.pub.publish(msg)


def read_key(timeout: float) -> str:
    """Espera una tecla hasta timeout segundos; devuelve '' si no hubo."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else ''


def main(args=None) -> None:
    """Punto de entrada: ros2 run octavius_bringup keyboard_teleop."""
    if not sys.stdin.isatty():
        print('keyboard_teleop necesita una terminal interactiva: '
              'ejecútalo con "ros2 run" en una terminal propia.')
        return

    rclpy.init(args=args)
    node = KeyboardTeleop()

    # Modo cbreak: cada tecla llega al instante (sin esperar Enter) y no se
    # muestra en pantalla. Ctrl+C sigue funcionando. Al salir se restaura
    # la configuración original de la terminal.
    original = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.1)
            if key:
                node.publish_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
