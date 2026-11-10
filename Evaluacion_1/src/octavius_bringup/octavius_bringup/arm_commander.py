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
Nodo arm_commander: une joystick y teclado en un solo comando para el brazo.

Problema que resuelve:
    arm_controller (JointGroupPositionController) necesita recibir SIEMPRE
    los 5 ángulos del brazo juntos. El joystick mueve base y hombro, y el
    teclado codo y muñeca: si cada uno publicara directo al controlador, uno
    sobrescribiría lo que mandó el otro. Este nodo guarda el vector objetivo
    completo y lo actualiza con ambas fuentes.

Entradas:
    /robot_description   (std_msgs/String)       límites de cada articulación
    /joint_states        (sensor_msgs/JointState) ángulos actuales, solo para
                                                  fijar el objetivo inicial
    /octavius/joystick   (sensor_msgs/Joy)  MODO VELOCIDAD, ~20 Hz (ESP32)
        axes[i] ∈ [-1, 1] → mueve joystick_joints[i] a v·ω_max
    /octavius/keyboard   (sensor_msgs/Joy)  MODO PASO, un mensaje por tecla
        axes[i] ∈ {-1, 0, +1} → mueve keyboard_joints[i] un paso fijo
        buttons[0] = 1        → alterna garra abierta / cerrada
        buttons[1] = 1        → vuelve suavemente a la pose home (todo en 0)

Salidas:
    /arm_controller/commands     (std_msgs/Float64MultiArray) 5 ángulos [rad]
    /gripper_controller/commands (std_msgs/Float64MultiArray) apertura de
                                 cada dedo [m], el mismo valor para ambos

Las fórmulas están en arm_logic.py. Los parámetros, en config/teleop.yaml.
"""

import signal
from typing import Dict, List, Optional

from octavius_bringup.arm_logic import (apply_deadzone, apply_step,
                                        integrate_velocity, parse_joint_limits,
                                        soft_limits, step_toward)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String


class ArmCommander(Node):
    """Integra los comandos de joystick y teclado y los envía al brazo."""

    def __init__(self) -> None:
        super().__init__('arm_commander')

        # ----- Parámetros (config/teleop.yaml) -----
        self.arm_joints: List[str] = self.declare_parameter(
            'arm_joints', ['base_joint', 'shoulder_joint', 'elbow_joint',
                           'wrist_pitch_joint', 'wrist_roll_joint']).value
        self.joystick_joints: List[str] = self.declare_parameter(
            'joystick_joints', ['base_joint', 'shoulder_joint']).value
        self.joystick_axis_sign: List[float] = self.declare_parameter(
            'joystick_axis_sign', [1.0, 1.0]).value
        self.keyboard_joints: List[str] = self.declare_parameter(
            'keyboard_joints', ['elbow_joint', 'wrist_pitch_joint',
                                'wrist_roll_joint']).value
        self.rate_hz: float = self.declare_parameter('rate_hz', 50.0).value
        self.max_velocity: float = self.declare_parameter(
            'joystick_max_velocity', 0.5).value
        self.deadzone: float = self.declare_parameter(
            'joystick_deadzone', 0.1).value
        self.joystick_timeout: float = self.declare_parameter(
            'joystick_timeout', 0.5).value
        self.keyboard_step: float = self.declare_parameter(
            'keyboard_step', 0.05).value
        self.home_velocity: float = self.declare_parameter(
            'home_velocity', 0.5).value
        self.limit_margin: float = self.declare_parameter(
            'limit_margin', 0.02).value
        self.gripper_joints: List[str] = self.declare_parameter(
            'gripper_joints', ['gripper_left_joint', 'gripper_right_joint']).value
        self.gripper_open: float = self.declare_parameter(
            'gripper_open', 0.02).value
        self.gripper_closed: float = self.declare_parameter(
            'gripper_closed', 0.0).value

        # ----- Estado interno -----
        self.limits: Dict[str, tuple] = {}
        self.target: Optional[Dict[str, float]] = None   # objetivo por junta
        self.joystick_axes: List[float] = [0.0] * len(self.joystick_joints)
        self.last_joystick_time: Optional[Time] = None
        self.gripper_is_open = False
        self.homing = False
        self.dt = 1.0 / self.rate_hz

        # ----- Comunicación -----
        # /robot_description se publica una sola vez con durabilidad
        # TRANSIENT_LOCAL ("latched"): hay que pedir la misma QoS para
        # recibir el mensaje aunque este nodo arranque después.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description',
                                 self.on_robot_description, latched)
        self.create_subscription(JointState, '/joint_states',
                                 self.on_joint_states, 10)
        self.create_subscription(Joy, '/octavius/joystick', self.on_joystick, 10)
        self.create_subscription(Joy, '/octavius/keyboard', self.on_keyboard, 10)

        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        self.create_timer(self.dt, self.on_timer)
        self.get_logger().info('arm_commander esperando /robot_description '
                               'y /joint_states...')

    # ------------------------------------------------------------------
    # Inicialización
    # ------------------------------------------------------------------
    def on_robot_description(self, msg: String) -> None:
        """Lee los límites del URDF y les aplica el margen (límites blandos)."""
        urdf_limits = parse_joint_limits(msg.data)
        missing = [j for j in self.arm_joints if j not in urdf_limits]
        if missing:
            self.get_logger().error(f'El URDF no define límites para: {missing}')
            return
        self.limits = {j: soft_limits(urdf_limits[j], self.limit_margin)
                       for j in self.arm_joints}

    def on_joint_states(self, msg: JointState) -> None:
        """
        Toma la posición actual como objetivo inicial (una sola vez).

        Si el objetivo partiera en 0, al primer comando el brazo saltaría
        desde donde esté hasta 0. Partiendo de la posición medida, no salta.
        """
        if self.target is not None or not self.limits:
            return
        measured = dict(zip(msg.name, msg.position))
        if not all(j in measured for j in self.arm_joints):
            return
        self.target = {j: measured[j] for j in self.arm_joints}
        self.gripper_is_open = (
            measured.get(self.gripper_joints[0], 0.0) > self.gripper_open / 2)
        self.get_logger().info('arm_commander listo: objetivo inicial = '
                               'posición actual del brazo.')

    # ------------------------------------------------------------------
    # Entradas
    # ------------------------------------------------------------------
    def on_joystick(self, msg: Joy) -> None:
        """Guarda la inclinación de cada eje (se aplica en on_timer)."""
        for i in range(len(self.joystick_joints)):
            axis = msg.axes[i] if i < len(msg.axes) else 0.0
            self.joystick_axes[i] = self.joystick_axis_sign[i] * apply_deadzone(
                axis, self.deadzone)
        self.last_joystick_time = self.get_clock().now()
        if any(self.joystick_axes):
            self.homing = False

    def on_keyboard(self, msg: Joy) -> None:
        """Aplica una pulsación de tecla: paso articular, garra o home."""
        if self.target is None:
            return
        for i, joint in enumerate(self.keyboard_joints):
            direction = msg.axes[i] if i < len(msg.axes) else 0.0
            if direction:
                self.homing = False
                self.target[joint] = apply_step(
                    self.target[joint], direction, self.keyboard_step,
                    self.limits[joint])

        buttons = list(msg.buttons) + [0, 0]
        if buttons[0]:
            self.gripper_is_open = not self.gripper_is_open
            self.publish_gripper()
        if buttons[1]:
            self.homing = True
            self.get_logger().info('Volviendo a la pose home...')

    # ------------------------------------------------------------------
    # Lazo principal
    # ------------------------------------------------------------------
    def on_timer(self) -> None:
        """Cada Δt: integra el joystick (o el homing) y publica el comando."""
        if self.target is None:
            return

        if self.homing:
            max_delta = self.home_velocity * self.dt
            for joint in self.arm_joints:
                self.target[joint] = step_toward(self.target[joint], 0.0,
                                                 max_delta)
            if all(v == 0.0 for v in self.target.values()):
                self.homing = False
                self.get_logger().info('Pose home alcanzada.')
        elif self.joystick_alive():
            for joint, axis in zip(self.joystick_joints, self.joystick_axes):
                self.target[joint] = integrate_velocity(
                    self.target[joint], axis, self.max_velocity, self.dt,
                    self.limits[joint])

        self.arm_pub.publish(Float64MultiArray(
            data=[self.target[j] for j in self.arm_joints]))

    def joystick_alive(self) -> bool:
        """
        Watchdog del joystick.

        Si no llegan datos en joystick_timeout segundos (cable desconectado,
        ESP32 reiniciada), se dejan de mover base y hombro en vez de seguir
        aplicando la última inclinación recibida.
        """
        if self.last_joystick_time is None:
            return False
        elapsed = (self.get_clock().now() - self.last_joystick_time).nanoseconds
        return elapsed * 1e-9 < self.joystick_timeout

    def publish_gripper(self) -> None:
        """Publica la apertura de la garra según su estado actual."""
        value = self.gripper_open if self.gripper_is_open else self.gripper_closed
        # Misma apertura para cada dedo: la garra es un solo GDL.
        self.gripper_pub.publish(
            Float64MultiArray(data=[value] * len(self.gripper_joints)))
        self.get_logger().info(
            f'Garra {"abierta" if self.gripper_is_open else "cerrada"}')


def main(args=None) -> None:
    """Punto de entrada: ros2 run octavius_bringup arm_commander."""
    # Cierre ordenado con Ctrl+C (o con el SIGINT que envía el launch).
    #
    # El manejador de señales NO lanza una excepción: solo levanta una
    # bandera que el lazo revisa cada 0.1 s. Una excepción podría aparecer
    # en cualquier punto, incluso dentro de un callback que rclpy está
    # ejecutando, y romper el cierre. Con la bandera el nodo siempre termina
    # en el mismo lugar, aunque SIGINT llegue dos veces (desde la terminal y
    # reenviado por el launch).
    #
    # Se desactiva el manejador de rclpy (que invalidaría el contexto en
    # medio de la espera) y se instala este explícitamente, porque un
    # proceso lanzado en segundo plano puede heredar SIGINT como "ignorada".
    stop_requested = False

    def on_sigint(signum, frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, on_sigint)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmCommander()
    # spin_once con timeout en vez de spin(): la espera vuelve a Python cada
    # 0.1 s para revisar la bandera. Con spin(), si Gazebo ya se cerró,
    # /clock se detiene, el timer (tiempo simulado) no vuelve a dispararse y
    # el nodo quedaría bloqueado.
    while rclpy.ok() and not stop_requested:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
