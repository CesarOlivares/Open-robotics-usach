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
Evidencia D12–D13: la IMU y la cámara publican y miden lo correcto.

1. Con el robot quieto, mide la frecuencia de /hexapod/imu,
   /hexapod/camera/image y /hexapod/camera/camera_info, y revisa la imagen
   (tamaño, frame_id) y la focal de camera_info.
2. Con la IMU quieta: gravedad medida ≈ 9,81 m/s² y cuerpo horizontal.
3. Hace girar el robot (ω = 0,4 rad/s) y compara el giro que mide la IMU
   (orientación y giroscopio) con la odometría de referencia.
4. Guarda una imagen de la cámara en docs/img/camara_frontal.png.

Uso (con la simulación corriendo):
    python3 scripts/verificar_sensores.py
"""

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
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, Imu  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img',
                   'camara_frontal.png')


def stamp(msg):
    """Tiempo del encabezado en segundos."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q):
    """Guiñada de un cuaternión."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tilt_of(q):
    """Inclinación total respecto de la horizontal [°]."""
    return math.degrees(2 * math.asin(min(1.0, math.hypot(q.x, q.y))))


class Verificador(Node):
    """Guarda los mensajes de los sensores."""

    def __init__(self):
        """Crea suscripciones y publicador."""
        super().__init__('verificar_sensores')
        self.msgs = {'imu': [], 'image': [], 'info': []}
        self.odom = None
        for key, typ, topic in (('imu', Imu, '/hexapod/imu'),
                                ('image', Image, '/hexapod/camera/image'),
                                ('info', CameraInfo, '/hexapod/camera/camera_info')):
            self.create_subscription(typ, topic, lambda m, k=key: self.msgs[k].append(m),
                                     qos_profile_sensor_data)
        self.create_subscription(Odometry, '/hexapod/odom',
                                 lambda m: setattr(self, 'odom', m), 10)
        self.pub = self.create_publisher(Twist, '/hexapod/cmd_vel', 10)

    def now(self):
        """Tiempo simulado según la odometría."""
        return stamp(self.odom) if self.odom else 0.0

    def run(self, seconds, wz=0.0):
        """Gira a wz durante `seconds` simulados (0 = quieto)."""
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t_end = self.now() + seconds
        while self.now() < t_end:
            msg = Twist()
            msg.angular.z = wz
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)


def rate(msgs):
    """Frecuencia media según los sellos de tiempo."""
    if len(msgs) < 2:
        return 0.0
    return (len(msgs) - 1) / (stamp(msgs[-1]) - stamp(msgs[0]))


def main():
    """Corre las verificaciones e imprime el resumen."""
    rclpy.init()
    n = Verificador()
    n.get_logger().info('esperando que el robot esté de pie...')
    while n.odom is None or n.odom.pose.pose.position.z < 0.095:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.run(2.0)
    for v in n.msgs.values():
        v.clear()
    n.run(5.0)
    ok = []

    imu, img, info = n.msgs['imu'], n.msgs['image'], n.msgs['info']
    print(f'IMU:        {rate(imu):6.1f} Hz  (esperado 100)  frame {imu[-1].header.frame_id}')
    print(f'Imagen:     {rate(img):6.1f} Hz  (esperado 15)   {img[-1].width}×{img[-1].height} '
          f'{img[-1].encoding}  frame {img[-1].header.frame_id}')
    print(f'CameraInfo: {rate(info):6.1f} Hz  fx = {info[-1].k[0]:.1f} px  (esperado ≈ 493)')
    ok += [abs(rate(imu) - 100) < 10, abs(rate(img) - 15) < 3,
           abs(info[-1].k[0] - 493) < 3, img[-1].header.frame_id == 'camera_optical_link']

    acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z] for m in imu])
    g = np.linalg.norm(acc.mean(axis=0))
    tilt = max(tilt_of(m.orientation) for m in imu)
    gyro_sd = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                        for m in imu]).std(axis=0).mean()
    print(f'Quieto:     |a| = {g:.3f} m/s² (gravedad)  inclinación máx {tilt:.3f}°  '
          f'ruido giroscopio σ = {gyro_sd:.4f} rad/s (configurado 0,005)')
    ok += [abs(g - 9.81) < 0.1, tilt < 1.0]

    # Guarda la imagen (R8G8B8 → arreglo alto × ancho × 3)
    m = img[-1]
    arr = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)
    arr = arr[:, :m.width * 3].reshape(m.height, m.width, 3)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    plt.imsave(OUT, arr)
    print(f'Imagen guardada: {os.path.abspath(OUT)}')

    # Giro: la IMU tiene que ver lo mismo que la odometría
    for v in n.msgs.values():
        v.clear()
    yaw_odom0 = yaw_of(n.odom.pose.pose.orientation)
    n.run(1.5, 0.4)  # arranque de la rampa y de la marcha
    imu_start = len(n.msgs['imu'])
    yaw_odom0, t0 = yaw_of(n.odom.pose.pose.orientation), n.now()
    n.run(5.0, 0.4)
    imu = n.msgs['imu'][imu_start:]
    yaw_imu = np.unwrap([yaw_of(m.orientation) for m in imu])
    gz = np.mean([m.angular_velocity.z for m in imu])
    d_odom = math.atan2(math.sin(yaw_of(n.odom.pose.pose.orientation) - yaw_odom0),
                        math.cos(yaw_of(n.odom.pose.pose.orientation) - yaw_odom0))
    d_imu = yaw_imu[-1] - yaw_imu[0]
    print(f'Giro 5 s:   odometría {math.degrees(d_odom):6.1f}°   IMU orientación '
          f'{math.degrees(d_imu):6.1f}°   giroscopio z medio {gz:+.3f} rad/s '
          f'(odometría {d_odom / (n.now() - t0):+.3f})')
    ok += [abs(d_imu - d_odom) < math.radians(3), d_imu > math.radians(20)]
    n.run(2.0)

    print('\nD12 + D13:', 'CUMPLE' if all(ok) else f'NO CUMPLE {ok}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
