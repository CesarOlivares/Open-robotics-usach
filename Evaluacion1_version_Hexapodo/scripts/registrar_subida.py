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
Registra cómo se pone de pie el robot y guarda docs/img/subida.png.

Lanzar ANTES que la simulación (o justo al lanzarla), porque registra desde
que llega el primer dato:
    python3 scripts/registrar_subida.py &
    ros2 launch hexapod_bringup gazebo.launch.py headless:=true use_rviz:=false

Grafica la altura del cuerpo (odometría de referencia) y el torque del fémur
y la tibia de la pata LF durante las fases recoger → parar → de pie.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img', 'subida.png')


def main():
    """Registra 16 s simulados desde el primer /joint_states y grafica."""
    rclpy.init()
    n = Node('registrar_subida')
    z, eff = [], []
    st = {'z': None}

    def on_odom(m):
        st['z'] = m.pose.pose.position.z

    def on_js(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st['z'] is None or t <= 0:
            return
        d = dict(zip(m.name, m.effort))
        z.append((t, st['z']))
        eff.append((t, abs(d['lf_femur_joint']), abs(d['lf_tibia_joint'])))

    n.create_subscription(Odometry, '/hexapod/odom', on_odom, 50)
    n.create_subscription(JointState, '/joint_states', on_js, 50)
    while rclpy.ok() and (not z or z[-1][0] - z[0][0] < 16.0):
        rclpy.spin_once(n, timeout_sec=0.1)
    # inicio de la fase "parar": cuando el cuerpo empieza a subir; el gráfico
    # parte 2 s antes (1 s de espera + 1 s de "recoger")
    t_up_abs = next(t for t, h in z if h > z[0][1] + 0.0005)
    t0 = t_up_abs - 2.0
    z = [(t, h) for t, h in z if t >= t0]
    eff = [e for e in eff if e[0] >= t0]
    t_up = 2.0
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.2), sharex=True)
    a1.plot([t - t0 for t, _ in z], [h * 1000 for _, h in z], color='#2464B0', lw=2)
    a1.set_ylabel('altura del cuerpo [mm]')
    a1.grid(alpha=0.25)
    a2.plot([t - t0 for t, _, _ in eff], [f for _, f, _ in eff], color='#2464B0', lw=1.6,
            label='fémur LF')
    a2.plot([t - t0 for t, _, _ in eff], [b for _, _, b in eff], color='#E58503', lw=1.6,
            label='tibia LF')
    a2.axhline(0.54, color='#C62828', ls='--', lw=1, label='torque de diseño (0,54)')
    a2.set_ylabel('torque [N·m]')
    a2.set_xlabel('tiempo [s]')
    a2.legend(fontsize=8)
    a2.grid(alpha=0.25)
    for ax in (a1, a2):
        ax.axvspan(t_up - 1.0, t_up, color='#9EA3AB', alpha=0.2)
        ax.axvspan(t_up, t_up + 2.5, color='#42A3FB', alpha=0.15)
    a1.text(t_up - 0.95, a1.get_ylim()[1] * 0.92, 'recoger', fontsize=9)
    a1.text(t_up + 0.1, a1.get_ylim()[1] * 0.92, 'parar (los pies bajan)', fontsize=9)
    a1.text(t_up + 2.65, a1.get_ylim()[1] * 0.92, 'de pie', fontsize=9)
    a1.set_title('El robot aparece apoyado en la batería y se para solo en 3,5 s. Al moverse,\n'
                 'el torque que informa Gazebo sube por el modo posición; quieto, baja al '
                 'valor estático', fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT, dpi=150)
    print(f'altura inicial {z[0][1] * 1000:.1f} mm, final {z[-1][1] * 1000:.1f} mm; '
          f'máx fémur {max(e[1] for e in eff):.3f}, tibia {max(e[2] for e in eff):.3f} N·m; '
          f'al final fémur {eff[-1][1]:.3f}, tibia {eff[-1][2]:.3f} N·m')
    print(f'gráfico: {os.path.abspath(OUT)}')


if __name__ == '__main__':
    main()
