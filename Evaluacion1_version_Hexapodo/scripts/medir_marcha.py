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
Evidencia D10–D11: el robot camina a la velocidad pedida (error < 20 %).

Para cada prueba: pide una velocidad en /hexapod/cmd_vel, deja pasar el
arranque y mide durante N segundos simulados el desplazamiento y el giro
reales (odometría de referencia /hexapod/odom). También registra el torque
máximo de cada tipo de articulación y la altura mínima del cuerpo.

Uso (con la simulación corriendo y el robot de pie):
    python3 scripts/medir_marcha.py                      # todas las pruebas
    python3 scripts/medir_marcha.py --pruebas adelante giro_izq --segundos 10
"""

import argparse
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# nombre → (vx [m/s], vy [m/s], wz [rad/s])
PRUEBAS = {
    'adelante': (0.08, 0.0, 0.0),
    'atras': (-0.08, 0.0, 0.0),
    'lateral_izq': (0.0, 0.06, 0.0),
    'giro_izq': (0.0, 0.0, 0.4),
    'giro_der': (0.0, 0.0, -0.4),
}


def yaw_of(q):
    """Ángulo de guiñada (en torno a z) de un cuaternión."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Medidor(Node):
    """Publica el comando y guarda odometría y torques."""

    def __init__(self):
        """Crea publicador y suscripciones."""
        super().__init__('medir_marcha')
        self.pub = self.create_publisher(Twist, '/hexapod/cmd_vel', 10)
        self.create_subscription(Odometry, '/hexapod/odom', self.on_odom, 50)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.odom = None
        self.raw_yaw = 0.0
        self.yaw_acc = 0.0
        self.effort = {}
        self.record = False

    def on_odom(self, m):
        """Guarda la última pose (t, x, y, z, yaw)."""
        p = m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        yaw = yaw_of(m.pose.pose.orientation)
        # Giro acumulado: el ángulo no "da la vuelta" al pasar ±π.
        if self.odom is not None:
            d = yaw - self.raw_yaw
            self.yaw_acc += math.atan2(math.sin(d), math.cos(d))
        self.raw_yaw = yaw
        self.odom = (t, p.x, p.y, p.z, self.yaw_acc)
        if self.record:
            self.zmin = min(self.zmin, p.z)

    def on_js(self, m):
        """Guarda el torque máximo por articulación mientras se registra."""
        if self.record:
            for name, e in zip(m.name, m.effort):
                self.effort.setdefault(name, []).append(abs(e))

    def send(self, vx, vy, wz):
        """Publica un Twist."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def wait_sim(self, seconds, cmd):
        """Publica cmd a 20 Hz durante `seconds` de tiempo simulado."""
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t_end = self.odom[0] + seconds
        while self.odom[0] < t_end:
            self.send(*cmd)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    """Corre las pruebas e imprime la tabla de resultados."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--pruebas', nargs='+', default=list(PRUEBAS), choices=list(PRUEBAS))
    ap.add_argument('--segundos', type=float, default=10.0)
    ap.add_argument('--arranque', type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    n = Medidor()
    n.get_logger().info('esperando que el robot esté de pie...')
    while n.odom is None or n.odom[3] < 0.095:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.wait_sim(2.0, (0.0, 0.0, 0.0))

    print(f'\n{"prueba":12s} {"pedido":>22s} {"medido":>22s} {"error":>7s}  '
          f'{"z mín":>7s}  torque coxa/fémur/tibia [N·m]: máximo y percentil 95')
    resultados = []
    for name in args.pruebas:
        cmd = PRUEBAS[name]
        n.wait_sim(args.arranque, cmd)
        t0, x0, y0, _, yaw0 = n.odom
        n.record, n.effort, n.zmin = True, {}, 1.0
        n.wait_sim(args.segundos, cmd)
        n.record = False
        t1, x1, y1, _, yaw1 = n.odom
        dt = t1 - t0
        dyaw = yaw1 - yaw0
        # Velocidad lineal medida en el marco del cuerpo (al inicio de la prueba)
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x1 - x0, y1 - y0
        meas = ((c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, dyaw / dt)
        if cmd[2] != 0.0:
            err = abs(meas[2] - cmd[2]) / abs(cmd[2])
            ped, med = f'wz={cmd[2]:+.3f} rad/s', f'wz={meas[2]:+.3f} rad/s'
        else:
            vc, vm = math.hypot(cmd[0], cmd[1]), (meas[0] * cmd[0] + meas[1] * cmd[1])
            err = abs(vm / vc - vc) / vc
            ped, med = f'v={vc:.3f} m/s', f'v={vm / vc:.3f} m/s'
        tq, p95 = [], []
        for seg in ('coxa', 'femur', 'tibia'):
            vals = sorted(v for k, lst in n.effort.items() if f'_{seg}_' in k for v in lst)
            tq.append(vals[-1])
            p95.append(vals[int(0.95 * (len(vals) - 1))])
        resultados.append(err < 0.20)
        print(f'{name:12s} {ped:>22s} {med:>22s} {err * 100:6.1f}%  '
              f'{n.zmin * 1000:5.1f}mm  máx {tq[0]:.2f}/{tq[1]:.2f}/{tq[2]:.2f}  '
              f'p95 {p95[0]:.2f}/{p95[1]:.2f}/{p95[2]:.2f}')
        n.wait_sim(2.0, (0.0, 0.0, 0.0))

    print('\nD11:', 'CUMPLE' if all(resultados) else 'NO CUMPLE',
          '(error de velocidad < 20 % en todas las pruebas)')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
