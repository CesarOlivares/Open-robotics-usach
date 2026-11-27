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
Lanza todo el proyecto con un solo comando.

    gazebo.launch.py         Gazebo, robot, controladores, gait_node, sensores, RViz
    joy_to_twist             joystick (/hexapod/joystick) → /hexapod/cmd_vel
    microros_agent.launch.py (si use_microros:=true) conecta la ESP32 por USB

El teclado (keyboard_teleop) va en otra terminal porque necesita leer el
teclado: ros2 run hexapod_control keyboard_teleop

Uso:
    ros2 launch hexapod_bringup bringup.launch.py
    ros2 launch hexapod_bringup bringup.launch.py use_microros:=true serial_port:=/dev/ttyUSB0
    ros2 launch hexapod_bringup bringup.launch.py use_microros:=true agent:=docker
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
    """Crea la descripción de lanzamiento completa."""
    bringup_share = get_package_share_directory('hexapod_bringup')
    launch_dir = os.path.join(bringup_share, 'launch')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz'),
                          'headless': LaunchConfiguration('headless'),
                          'world': LaunchConfiguration('world')}.items(),
    )

    joy_to_twist = Node(
        package='hexapod_control',
        executable='joy_to_twist',
        parameters=[os.path.join(bringup_share, 'config', 'joystick.yaml'),
                    {'use_sim_time': True}],
        output='screen',
    )

    microros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'microros_agent.launch.py')),
        launch_arguments={'serial_port': LaunchConfiguration('serial_port'),
                          'agent': LaunchConfiguration('agent')}.items(),
        condition=IfCondition(LaunchConfiguration('use_microros')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Abrir RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo sin interfaz gráfica'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            bringup_share, 'worlds', 'hexapod_world.sdf'), description='Mundo de Gazebo'),
        DeclareLaunchArgument('use_microros', default_value='false',
                              description='Lanzar el agente micro-ROS (joystick ESP32)'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Puerto serial de la ESP32'),
        DeclareLaunchArgument('agent', default_value='native',
                              description='Agente micro-ROS: native o docker'),
        gazebo,
        joy_to_twist,
        microros_agent,
    ])
