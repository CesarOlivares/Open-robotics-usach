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
Evidencia D8: el robot se mantiene de pie (altura estable ± 2 mm).

Lee la odometría de referencia (/hexapod/odom) y el torque de las
articulaciones (/joint_states) durante N segundos de tiempo simulado,
desde que el cuerpo pasa la altura de inicio, e imprime un resumen.

Uso (con la simulación corriendo):
    python3 scripts/medir_postura.py --segundos 60
"""

import argparse
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main():
    """Mide altura, inclinación, deriva y torque máximo."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--segundos', type=float, default=60.0)
    ap.add_argument('--desde_altura', type=float, default=0.050,
                    help='empieza a registrar cuando base_link supera esta altura [m]')
    ap.add_argument('--asentamiento', type=float, default=5.0,
                    help='segundos desde ese instante que se excluyen de la '
                         'estabilidad (la subida termina en ~3 s)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('medir_postura')
    z, tilt, xy, t = [], [], [], []
    effort = {}
    state = {'t0': None}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if state['t0'] is None:
            if p.z < args.desde_altura:
                return
            state['t0'] = now
        t.append(now - state['t0'])
        z.append(p.z)
        xy.append(math.hypot(p.x, p.y))
        tilt.append(math.degrees(2 * math.asin(min(1.0, math.hypot(q.x, q.y)))))

    def on_js(m):
        if state['t0'] is None:
            return
        for name, e in zip(m.name, m.effort):
            effort[name] = max(effort.get(name, 0.0), abs(e))

    node.create_subscription(Odometry, '/hexapod/odom', on_odom, 50)
    node.create_subscription(JointState, '/joint_states', on_js, 50)
    total = args.segundos + args.asentamiento
    while rclpy.ok() and (not t or t[-1] < total):
        rclpy.spin_once(node, timeout_sec=0.5)

    steady = [i for i, ti in enumerate(t) if ti >= args.asentamiento]
    z = [z[i] for i in steady]
    tilt = [tilt[i] for i in steady]
    xy = [xy[i] for i in steady]
    zmed = sum(z) / len(z)
    print(f'estabilidad: {len(z)} muestras en {t[-1] - args.asentamiento:.1f} s simulados '
          f'(después de {args.asentamiento:.0f} s de asentamiento)')
    print(f'altura base_link: media {zmed * 1000:.2f} mm, '
          f'rango [{min(z) * 1000:.2f}, {max(z) * 1000:.2f}] mm, '
          f'variación máx ±{max(abs(v - zmed) for v in z) * 1000:.2f} mm')
    print(f'inclinación máx: {max(tilt):.3f}°   deriva horizontal máx: {max(xy) * 1000:.2f} mm')
    for seg in ('coxa', 'femur', 'tibia'):
        vals = [v for k, v in effort.items() if f'_{seg}_' in k]
        print(f'torque máx {seg:5s} (incluye la subida): {max(vals):.3f} N·m')
    ok = max(abs(v - zmed) for v in z) <= 0.002
    print('D8:', 'CUMPLE' if ok else 'NO CUMPLE', '(altura estable ± 2 mm)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
