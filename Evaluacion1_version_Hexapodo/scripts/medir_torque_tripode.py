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
Torque estático con el robot apoyado en 3 patas (el peor caso de la marcha).

Caminando, el torque que reporta gz_ros2_control no sirve como carga: en
modo posición impone velocidades y las patas apoyadas se empujan entre sí
al máximo. Quieto en 3 patas, en cambio, el torque es solo el que exige el
peso, y se puede comparar con el cálculo a mano.

Pasos: detiene gait_node, levanta el grupo B (rf, lm, rr) `--levantar` m y
mantiene la pose unos segundos; luego imprime el torque de cada articulación.

Uso (con la simulación corriendo y el robot de pie):
    python3 scripts/medir_torque_tripode.py
    python3 scripts/medir_torque_tripode.py --radial 0.09   # pie más afuera
Al terminar, relanzar la simulación (gait_node quedó detenido).
"""

import argparse
import subprocess
import time

from hexapod_control.robot_model import HexapodModel, LEG_NAMES
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def main():
    """Pone el robot en 3 patas y mide."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--levantar', type=float, default=0.03)
    ap.add_argument('--radial', type=float, default=0.070,
                    help='pie de apoyo: distancia horizontal desde el eje del fémur [m]')
    ap.add_argument('--altura', type=float, default=0.090)
    ap.add_argument('--segundos', type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node('medir_torque_tripode')
    state = {'urdf': None, 'eff': {}, 'z': 0.0}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/robot_description',
                             lambda m: state.__setitem__('urdf', m.data), qos)
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state['eff'].update(zip(m.name, m.effort)), 10)
    node.create_subscription(Odometry, '/hexapod/odom',
                             lambda m: state.__setitem__('z', m.pose.pose.position.z), 10)
    # Espera el URDF y que el robot esté de pie (gait_node terminó de pararlo).
    while state['urdf'] is None or state['z'] < 0.095:
        rclpy.spin_once(node, timeout_sec=0.1)
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    model = HexapodModel(state['urdf'])
    l1 = model.legs[0].geom.coxa
    feet = model.neutral_feet(l1 + args.radial, args.altura)
    for i, name in enumerate(LEG_NAMES):
        if name in ('rf', 'lm', 'rr'):
            feet[i] = (feet[i][0], feet[i][1], feet[i][2] + args.levantar)
    angles = model.joint_angles(feet)

    subprocess.run(['pkill', '-f', 'hexapod_control/[g]ait_node'], check=False)
    pub = node.create_publisher(Float64MultiArray, '/legs_controller/commands', 10)
    t0 = time.time()
    while time.time() - t0 < args.segundos:
        pub.publish(Float64MultiArray(data=angles))
        rclpy.spin_once(node, timeout_sec=0.02)

    print(f'apoyo en lf, rm, lr; pie a {args.radial * 1000:.0f} mm del fémur; '
          f'cuerpo a {state["z"] * 1000:.1f} mm')
    print('pata   coxa    fémur   tibia  [N·m]')
    for name in LEG_NAMES:
        e = [state['eff'][f'{name}_{s}_joint'] for s in ('coxa', 'femur', 'tibia')]
        print(f'{name}   {e[0]:+.3f}  {e[1]:+.3f}  {e[2]:+.3f}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
