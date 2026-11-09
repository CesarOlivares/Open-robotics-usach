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
Lanza el agente micro-ROS que conecta la ESP32 (joystick) con ROS 2.

Qué es el agente:
    La ESP32 no puede correr DDS (el middleware de ROS 2): es demasiado
    pesado para un microcontrolador. micro-ROS usa en la placa el protocolo
    XRCE-DDS, mucho más liviano, y el agente corre en el PC traduciendo entre
    XRCE-DDS (por el cable USB serial) y DDS. Para el resto de ROS 2 la ESP32
    aparece como un nodo más (/octavius_joystick).

Dos formas de ejecutarlo (argumento agent):
    native  paquete micro_ros_agent compilado desde fuente, como indica el
            tutorial oficial de micro-ROS (ver README, sección micro-ROS).
    docker  imagen oficial microros/micro-ros-agent:jazzy; no requiere
            compilar nada, solo Docker.

Uso:
    ros2 launch octavius_bringup microros_agent.launch.py
    ros2 launch octavius_bringup microros_agent.launch.py serial_port:=/dev/ttyACM0
    ros2 launch octavius_bringup microros_agent.launch.py agent:=docker
"""

from ament_index_python.packages import (get_package_share_directory,
                                         PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DOCKER_IMAGE = 'microros/micro-ros-agent:jazzy'


def create_agent(context):
    """Elige cómo lanzar el agente según el argumento agent."""
    mode = LaunchConfiguration('agent').perform(context)
    port = LaunchConfiguration('serial_port').perform(context)
    baudrate = LaunchConfiguration('baudrate').perform(context)
    # Argumentos del agente: transporte serial, puerto, velocidad y nivel de
    # detalle del log (4 = muestra cuando la ESP32 se conecta o desconecta).
    agent_args = ['serial', '--dev', port, '-b', baudrate, '-v4']

    if mode == 'docker':
        # --net=host: comparte la red del PC para que DDS descubra los nodos.
        # --device: da acceso al puerto serial dentro del contenedor.
        return [ExecuteProcess(
            cmd=['docker', 'run', '--rm', '--net=host',
                 f'--device={port}', DOCKER_IMAGE] + agent_args,
            output='screen')]

    if mode != 'native':
        raise RuntimeError(f"agent:='{mode}' no es válido: usar 'native' o 'docker'.")

    try:
        get_package_share_directory('micro_ros_agent')
    except PackageNotFoundError:
        raise RuntimeError(
            'No se encontró el paquete micro_ros_agent. Hacer "source" del '
            'workspace donde se compiló (ver README) o usar agent:=docker.')

    return [Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=agent_args,
        output='screen')]


def generate_launch_description():
    """Crea la descripción de lanzamiento del agente micro-ROS."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Puerto serial de la ESP32 (ver "ls /dev/ttyUSB* /dev/ttyACM*")'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Velocidad del puerto; debe coincidir con el firmware'),
        DeclareLaunchArgument(
            'agent', default_value='native',
            description='native: paquete micro_ros_agent compilado; '
                        'docker: imagen oficial de micro-ROS'),
        OpaqueFunction(function=create_agent),
    ])
