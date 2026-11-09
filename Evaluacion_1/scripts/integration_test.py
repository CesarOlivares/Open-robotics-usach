"""
Prueba de integración del control de Octavius (teclado y joystick).

Requiere la simulación en marcha:
    ros2 launch octavius_bringup bringup.launch.py

Luego, en otra terminal:
    source install/setup.bash
    python3 scripts/integration_test.py

Envía comandos como lo harían keyboard_teleop y la ESP32 (sensor_msgs/Joy)
y muestra las posiciones medidas en /joint_states junto al valor esperado.
"""
import time

import rclpy
from sensor_msgs.msg import JointState, Joy

JOINTS = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint',
          'wrist_roll_joint', 'gripper_joint']

rclpy.init()
node = rclpy.create_node('fase3_test')
state = {}
node.create_subscription(JointState, '/joint_states',
                         lambda m: state.update(zip(m.name, m.position)), 10)
kb = node.create_publisher(Joy, '/octavius/keyboard', 10)
js = node.create_publisher(Joy, '/octavius/joystick', 10)


def spin(seconds):
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


def show(title):
    spin(0.3)
    print(title)
    print('   ' + '  '.join(f'{j.replace("_joint", "")}={state.get(j, float("nan")):+.3f}'
                          for j in JOINTS))


def key(axes, buttons=(0, 0), times=1):
    for _ in range(times):
        kb.publish(Joy(axes=list(axes), buttons=list(buttons)))
        spin(0.05)


def joystick(axes, seconds, rate=20):
    for _ in range(int(seconds * rate)):
        js.publish(Joy(axes=list(axes)))
        spin(1.0 / rate)


# Esperar a que existan suscriptores (arm_commander)
while kb.get_subscription_count() == 0 or js.get_subscription_count() == 0:
    spin(0.2)
spin(2.0)
show('1) Estado inicial')

key((1.0, 0.0, 0.0), times=6)
key((0.0, 1.0, 0.0), times=4)
key((0.0, 0.0, 1.0), times=3)
key((0.0, 0.0, 0.0), buttons=(1, 0))
spin(2.0)
show('2) Teclado: 6×w, 4×r, 3×e, a  → esperado codo +0.30, pitch +0.20, roll +0.15, garra 0.02')

joystick((1.0, -0.6), 2.0)
spin(1.5)
show('3) Joystick X=+1, Y=-0.6 por 2 s → esperado base ≈ +1.0, hombro ≈ -0.6 '
     '(y quieto tras soltar: watchdog)')

joystick((0.05, 0.05), 1.5)
spin(1.0)
show('4) Joystick 0.05 (bajo zona muerta 0.1) → sin cambios')

key((1.0, 0.0, 0.0), times=100)
spin(3.0)
show('5) Codo con 100 pulsaciones → saturado en el límite 2.356')

key((0.0, 0.0, 0.0), buttons=(0, 1))
spin(7.0)
show('6) Home (h) → todo vuelve a 0 (la garra no cambia)')

node.destroy_node()
rclpy.shutdown()
