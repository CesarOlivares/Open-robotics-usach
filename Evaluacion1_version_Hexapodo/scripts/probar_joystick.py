#!/usr/bin/env python3
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
Evidencia D15: el joystick (simulado) mueve al hexápodo con control suave.

Hace lo mismo que hará la ESP32: publica sensor_msgs/Joy en /hexapod/joystick
a 20 Hz. Mueve la palanca de a poco (eje Y de 0 a 1 en `--segundos`), la
mantiene al tope y la suelta. Registra:
    - la posición de la palanca,
    - la velocidad que pide joy_to_twist (/hexapod/cmd_vel),
    - la velocidad real del cuerpo (odometría de referencia, media móvil de
      un ciclo de marcha para quitar el vaivén de cada paso).
Guarda el gráfico en docs/img/joystick_gradiente.png.

Uso (con bringup.launch.py corriendo, que incluye joy_to_twist):
    python3 scripts/probar_joystick.py
"""

import argparse
import math
import os

from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img',
                   'joystick_gradiente.png')


class Prueba(Node):
    """Joystick simulado + registro."""

    def __init__(self):
        """Publicador del joystick y suscripciones."""
        super().__init__('probar_joystick')
        self.pub = self.create_publisher(Joy, '/hexapod/joystick', 10)
        self.cmd = 0.0
        self.odom = None
        self.create_subscription(Twist, '/hexapod/cmd_vel',
                                 lambda m: setattr(self, 'cmd', m.linear.x), 10)
        self.create_subscription(Odometry, '/hexapod/odom',
                                 lambda m: setattr(self, 'odom', m), 50)

    def t(self):
        """Tiempo simulado."""
        s = self.odom.header.stamp
        return s.sec + s.nanosec * 1e-9


def main():
    """Barrido de la palanca y gráfico."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--segundos', type=float, default=20.0, help='duración de la subida')
    args = ap.parse_args()

    rclpy.init()
    n = Prueba()
    while n.odom is None or n.odom.pose.pose.position.z < 0.095:
        rclpy.spin_once(n, timeout_sec=0.1)
    t0 = n.t() + 1.0
    rise, hold, rest = args.segundos, 6.0, 4.0
    rows = []
    next_pub = 0.0
    while n.t() < t0 + rise + hold + rest:
        rclpy.spin_once(n, timeout_sec=0.01)
        t = n.t() - t0
        if t < 0:
            y = 0.0
        elif t < rise:
            y = t / rise
        elif t < rise + hold:
            y = 1.0
        else:
            y = 0.0
        if n.t() >= next_pub:                    # 20 Hz, como la ESP32
            msg = Joy()
            msg.axes = [0.0, y]
            msg.buttons = [0]
            n.pub.publish(msg)
            next_pub = n.t() + 0.05
        p = n.odom.pose.pose.position
        rows.append((t, y, n.cmd, p.x, p.y))

    a = np.array(rows)
    a = a[a[:, 0] >= 0]
    # Velocidad real: avance en una ventana de 1 s (un ciclo de marcha)
    t, x = a[:, 0], a[:, 3]
    v = np.full_like(t, np.nan)
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > 1.0:
            j += 1
        if t[i] - t[j] > 0.9:
            v[i] = (x[i] - x[j]) / (t[i] - t[j])
    t_mid = t - 0.5

    fig, ax1 = plt.subplots(figsize=(10, 4.8))
    ax1.plot(a[:, 0], a[:, 1], color='#9EA3AB', lw=1.5, label='palanca (eje Y, 0 a 1)')
    ax1.set_ylabel('posición de la palanca')
    ax1.set_xlabel('tiempo [s]')
    ax2 = ax1.twinx()
    ax2.plot(a[:, 0], a[:, 2] * 100, color='#EA7600', lw=2, label='velocidad pedida (joy_to_twist)')
    ax2.plot(t_mid, v * 100, color='#002F6C', lw=2, label='velocidad real (media de 1 ciclo)')
    ax2.set_ylabel('velocidad de avance [cm/s]')
    ax2.set_ylim(-3, 11)
    ax1.set_ylim(-0.3, 1.1)
    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax1.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=9)
    ax1.grid(alpha=0.25)
    ax1.set_title('Joystick → velocidad del hexápodo: zona muerta 8 %, curva expo 0,5',
                  color='#002F6C')
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)

    hold_mask = (a[:, 0] > rise + 2) & (a[:, 0] < rise + hold)
    v_hold = np.nanmean(v[(t > rise + 2) & (t < rise + hold)])
    cmd_hold = a[hold_mask, 2].mean()
    total = math.hypot(a[-1, 3] - a[0, 3], a[-1, 4] - a[0, 4])
    print(f'avance total: {total * 100:.1f} cm')
    print(f'palanca al tope: pedido {cmd_hold * 100:.2f} cm/s, real {v_hold * 100:.2f} cm/s')
    after = t > rise + hold
    x_rel = x[after][np.argmax(a[after, 2] < 1e-6):]
    back = (x_rel.max() - x_rel[-1]) * 100 if len(x_rel) else 0.0
    print(f'palanca suelta: pedido {a[-1, 2] * 100:.2f} cm/s; tras soltar el cuerpo '
          f'retrocede {back:.2f} cm mientras los pies vuelven a su lugar')
    print(f'gráfico: {os.path.abspath(OUT)}')
    print('D15:', 'CUMPLE' if total > 0.3 and abs(a[-1, 2]) < 1e-6 else 'NO CUMPLE')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
