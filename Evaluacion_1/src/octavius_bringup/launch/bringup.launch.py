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
Launch principal de Octavius: simulación + control por joystick y teclado.

Incluye:
    gazebo.launch.py          Gazebo, robot, controladores, sensores y RViz
    arm_commander             une joystick y teclado en el comando del brazo
    microros_agent.launch.py  (con use_microros:=true) agente micro-ROS
                              que conecta la ESP32 con el joystick

El teclado NO se lanza aquí: necesita leer una terminal, así que se abre
aparte (igual que teleop_twist_keyboard en la documentación oficial):
    ros2 run octavius_bringup keyboard_teleop

Uso:
    ros2 launch octavius_bringup bringup.launch.py
    ros2 launch octavius_bringup bringup.launch.py use_microros:=true
    ros2 launch octavius_bringup bringup.launch.py use_microros:=true serial_port:=/dev/ttyACM0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Crea la descripción de lanzamiento completa de Octavius."""
    bringup_share = get_package_share_directory('octavius_bringup')
    teleop_file = os.path.join(bringup_share, 'config', 'teleop.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )

    microros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'microros_agent.launch.py')),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'agent': LaunchConfiguration('agent'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_microros')),
    )

    arm_commander = Node(
        package='octavius_bringup',
        executable='arm_commander',
        parameters=[teleop_file, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_share, 'worlds', 'workcell.sdf'),
            description='Ruta del mundo .sdf de Gazebo'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Abrir RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo sin interfaz gráfica'),
        DeclareLaunchArgument(
            'use_microros', default_value='false',
            description='Lanzar el agente micro-ROS para el joystick de la ESP32'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Puerto serial de la ESP32'),
        DeclareLaunchArgument(
            'agent', default_value='native',
            description='Agente micro-ROS: native (compilado) o docker'),
        simulation,
        arm_commander,
        microros_agent,
    ])
