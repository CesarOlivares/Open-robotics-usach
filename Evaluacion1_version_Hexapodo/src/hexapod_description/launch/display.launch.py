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
Visualiza el hexápodo (o una sola pata) en RViz con sliders.

No usa Gazebo ni ros2_control: sirve para revisar el modelo (pauta P3, P4, P5).

Nodos:
    robot_state_publisher      lee el URDF (parámetro robot_description) y,
                               con los ángulos de /joint_states, publica la
                               TF de cada link en /tf y /tf_static.
    joint_state_publisher_gui  un slider por articulación; publica
                               /joint_states.
    rviz2                      dibuja RobotModel y TF.

Uso:
    ros2 launch hexapod_description display.launch.py                  (hexápodo)
    ros2 launch hexapod_description display.launch.py model:=leg_test  (una pata)
    ros2 launch hexapod_description display.launch.py use_gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Crea la descripción de lanzamiento de la visualización en RViz."""
    pkg_share = get_package_share_directory('hexapod_description')
    rviz_file = os.path.join(pkg_share, 'rviz', 'display.rviz')

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')

    # model:=leg_test → urdf/leg_test.urdf.xacro ; model:=hexapod → urdf/hexapod.urdf.xacro
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', [model, '.urdf.xacro']])

    # El .xacro se procesa al lanzar: el resultado (URDF en texto) es el
    # parámetro robot_description de robot_state_publisher.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='hexapod',
            description='leg_test: una pata sobre un soporte (E1); hexapod: robot completo'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='true: sliders (joint_state_publisher_gui); '
                        'false: articulaciones fijas en 0'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(use_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])
