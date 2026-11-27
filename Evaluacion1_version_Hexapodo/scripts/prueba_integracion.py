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
Prueba de integración del sistema completo (E7).

Con bringup.launch.py corriendo, recorre toda la cadena como lo haría un
usuario y comprueba cada punto con un criterio numérico:

    1. Articulaciones: /joint_states trae las 18 juntas.
    2. Postura: el robot se para solo y queda a su altura de diseño ± 2 mm.
    3. Sensores: IMU ~100 Hz, cámara ~15 Hz.
    4. Joystick (como la ESP32): palanca adelante → avanza.
    5. Joystick: palanca a la derecha → gira a la derecha.
    6. Joystick: botón y palanca a la derecha → paso lateral a la derecha.
    7. Teclado (Twist, como keyboard_teleop): atrás → retrocede.
    8. Seguridad: al dejar de publicar, el robot se detiene solo.

Termina con código 0 si todo cumple y 1 si algo falla (sirve para la
prueba en limpio).

Uso:
    ros2 launch hexapod_bringup bringup.launch.py headless:=true use_rviz:=false
    python3 scripts/prueba_integracion.py           # en otra terminal
"""

import math
import os
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, Joy
import yaml


def expected_height():
    """Altura de base_link de pie: stand_height (gait.yaml) + radio del pie."""
    with open(os.path.join(get_package_share_directory('hexapod_bringup'),
                           'config', 'gait.yaml')) as f:
        h = yaml.safe_load(f)['gait_node']['ros__parameters']['stand_height']
    params = ET.parse(os.path.join(get_package_share_directory('hexapod_description'),
                                   'urdf', 'hexapod_params.xacro')).getroot()
    r = next(float(p.get('value')) for p in params if p.get('name') == 'foot_radius')
    return h + r


def yaw_of(q):
    """Guiñada de un cuaternión."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Prueba:
    """Publicadores, suscripciones y utilidades de tiempo simulado."""

    def __init__(self):
        """Crea el nodo de prueba."""
        self.node = rclpy.create_node('prueba_integracion')
        n = self.node
        self.odom = None
        self.joints = []
        self.count = {'imu': [], 'image': []}
        n.create_subscription(Odometry, '/hexapod/odom', self._odom, 50)
        n.create_subscription(JointState, '/joint_states',
                              lambda m: setattr(self, 'joints', list(m.name)), 10)
        n.create_subscription(Imu, '/hexapod/imu', lambda m: self._stamp('imu', m),
                              qos_profile_sensor_data)
        n.create_subscription(Image, '/hexapod/camera/image',
                              lambda m: self._stamp('image', m), qos_profile_sensor_data)
        self.joy = n.create_publisher(Joy, '/hexapod/joystick', 10)
        self.twist = n.create_publisher(Twist, '/hexapod/cmd_vel', 10)
        self.raw_yaw, self.yaw = None, 0.0

    def _odom(self, m):
        y = yaw_of(m.pose.pose.orientation)
        if self.raw_yaw is not None:
            d = y - self.raw_yaw
            self.yaw += math.atan2(math.sin(d), math.cos(d))
        self.raw_yaw = y
        self.odom = m

    def _stamp(self, key, m):
        self.count[key].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def t(self):
        """Tiempo simulado."""
        s = self.odom.header.stamp
        return s.sec + s.nanosec * 1e-9

    def pose(self):
        """(x, y, z, yaw acumulado) del cuerpo en el mundo."""
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z, self.yaw

    def spin_until(self, cond, timeout=120.0):
        """Procesa mensajes hasta que cond() sea verdadero (o timeout en s reales)."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if cond():
                return True
        return False

    def run(self, seconds, joy=None, twist=None):
        """Publica joy y/o twist a 20 Hz durante `seconds` simulados."""
        t_end = self.t() + seconds
        next_pub = 0.0
        while self.t() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.t() >= next_pub:
                if joy is not None:
                    self.joy.publish(Joy(axes=list(joy[0]), buttons=list(joy[1])))
                if twist is not None:
                    msg = Twist()
                    msg.linear.x, msg.linear.y, msg.angular.z = twist
                    self.twist.publish(msg)
                next_pub = self.t() + 0.05


def in_body(dx, dy, yaw):
    """Desplazamiento del mundo expresado en el marco del cuerpo."""
    return (math.cos(yaw) * dx + math.sin(yaw) * dy,
            -math.sin(yaw) * dx + math.cos(yaw) * dy)


def main():
    """Corre las 8 comprobaciones e imprime la tabla."""
    rclpy.init()
    p = Prueba()
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print(f'{"CUMPLE   " if ok else "NO CUMPLE"}  {name:42s} {detail}', flush=True)

    z_ok = expected_height()
    print(f'Esperando la simulación y que el robot se pare ({z_ok * 1000:.0f} mm)...',
          flush=True)
    stood = p.spin_until(lambda: p.odom is not None
                         and p.odom.pose.pose.position.z > z_ok - 0.002)
    if not stood:
        print('NO CUMPLE  el robot no se paró en 120 s (¿está corriendo bringup.launch.py?)')
        sys.exit(1)
    p.run(3.0)

    check('1. /joint_states con 18 articulaciones', len(p.joints) == 18,
          f'{len(p.joints)} juntas')
    z = p.pose()[2]
    check(f'2. De pie a {z_ok * 1000:.0f} ± 2 mm', abs(z - z_ok) < 0.002, f'{z * 1000:.1f} mm')

    for v in p.count.values():
        v.clear()
    p.run(4.0)
    rate = {k: (len(v) - 1) / (v[-1] - v[0]) if len(v) > 1 else 0.0
            for k, v in p.count.items()}
    check('3. IMU ~100 Hz y cámara ~15 Hz',
          abs(rate['imu'] - 100) < 10 and abs(rate['image'] - 15) < 3,
          f'IMU {rate["imu"]:.1f} Hz, cámara {rate["image"]:.1f} Hz')

    x0, y0, _, yaw0 = p.pose()
    p.run(6.0, joy=((0.0, 1.0), (0,)))
    x1, y1, _, yaw1 = p.pose()
    fwd, side = in_body(x1 - x0, y1 - y0, yaw0)
    check('4. Joystick adelante → avanza', fwd > 0.35 and abs(side) < 0.1,
          f'avanzó {fwd * 100:.1f} cm (lateral {side * 100:+.1f} cm) en 6 s')
    p.run(1.5, joy=((0.0, 0.0), (0,)))

    _, _, _, yaw0 = p.pose()
    p.run(5.0, joy=((1.0, 0.0), (0,)))
    _, _, _, yaw1 = p.pose()
    dyaw = math.degrees(yaw1 - yaw0)
    check('5. Joystick derecha → gira a la derecha', dyaw < -45,
          f'giró {dyaw:+.0f}° en 5 s')
    p.run(1.5, joy=((0.0, 0.0), (0,)))

    p.run(0.3, joy=((0.0, 0.0), (1,)))      # apretar el botón: modo lateral
    p.run(0.3, joy=((0.0, 0.0), (0,)))
    x0, y0, _, yaw0 = p.pose()
    p.run(6.0, joy=((1.0, 0.0), (0,)))
    x1, y1, _, _ = p.pose()
    fwd, side = in_body(x1 - x0, y1 - y0, yaw0)
    check('6. Botón + derecha → paso lateral derecho', side < -0.3 and abs(fwd) < 0.1,
          f'lateral {side * 100:+.1f} cm (avance {fwd * 100:+.1f} cm) en 6 s')
    p.run(0.3, joy=((0.0, 0.0), (1,)))      # vuelve al modo giro
    p.run(1.5, joy=((0.0, 0.0), (0,)))

    x0, y0, _, yaw0 = p.pose()
    p.run(5.0, twist=(-0.08, 0.0, 0.0))
    x1, y1, _, _ = p.pose()
    fwd, _ = in_body(x1 - x0, y1 - y0, yaw0)
    check('7. Teclado (Twist) atrás → retrocede', fwd < -0.25,
          f'{fwd * 100:+.1f} cm en 5 s')

    p.run(4.0, twist=(0.08, 0.0, 0.0))
    p.run(2.0)                              # nadie publica: debe detenerse solo
    x0, y0, _, _ = p.pose()
    p.run(3.0)
    x1, y1, _, _ = p.pose()
    moved = math.hypot(x1 - x0, y1 - y0)
    check('8. Sin comandos se detiene solo', moved < 0.01,
          f'se movió {moved * 1000:.1f} mm en 3 s')

    ok = all(results)
    print(f'\nRESULTADO: {sum(results)}/{len(results)} '
          + ('— TODO CUMPLE' if ok else '— HAY FALLAS'))
    p.node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
