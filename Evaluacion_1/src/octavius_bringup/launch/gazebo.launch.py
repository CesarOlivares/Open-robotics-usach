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
Simula Octavius en Gazebo Harmonic con ros2_control, sensores y RViz.

Secuencia:
    1. Gazebo abre el mundo (workcell.sdf).
    2. robot_state_publisher publica el URDF en /robot_description.
    3. ros_gz_sim create toma /robot_description y aparece el brazo sobre la
       mesa. Al cargarse, el plugin gz_ros2_control levanta el
       controller_manager dentro de Gazebo.
    4. Cuando create termina, se activa joint_state_broadcaster; cuando este
       termina, arm_controller y gripper_controller. El orden se fuerza con
       OnProcessExit para que nunca se intente activar un controlador antes
       de que exista el robot.
    5. ros_gz_bridge pasa /clock, la cámara y la IMU de Gazebo a ROS.
    6. RViz muestra el modelo, la TF y la imagen de la cámara.

Uso:
    ros2 launch octavius_bringup gazebo.launch.py
    ros2 launch octavius_bringup gazebo.launch.py use_rviz:=false
    ros2 launch octavius_bringup gazebo.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Altura de la superficie de la mesa en workcell.sdf [m].
TABLE_HEIGHT = '0.75'


def generate_launch_description():
    """Crea la descripción de lanzamiento de la simulación en Gazebo."""
    description_share = get_package_share_directory('octavius_description')
    bringup_share = get_package_share_directory('octavius_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'octavius.urdf.xacro')
    controllers_file = os.path.join(bringup_share, 'config', 'controllers.yaml')
    bridge_file = os.path.join(bringup_share, 'config', 'gz_bridge.yaml')
    rviz_file = os.path.join(bringup_share, 'config', 'bringup.rviz')
    default_world = os.path.join(bringup_share, 'worlds', 'workcell.sdf')

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # URDF procesado, con la ruta del YAML de controladores para el plugin
    # gz_ros2_control.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers_file]),
        value_type=str)

    # -r: arranca la simulación sin pausa.  -v 2: solo errores y advertencias.
    # En modo headless (-s) no se abre la interfaz gráfica de Gazebo.
    gz_args = [
        '-r -v 2 ', world,
        PythonExpression(["' -s --headless-rendering' if '", headless,
                          "' == 'true' else ''"]),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'octavius',
                   '-z', TABLE_HEIGHT],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_file}],
        output='screen',
    )

    def spawner(controller):
        """Nodo que carga y activa un controlador en el controller_manager."""
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    arm_controller = spawner('arm_controller')
    gripper_controller = spawner('gripper_controller')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Ruta del mundo .sdf de Gazebo'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Abrir RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo sin interfaz gráfica'),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,

        # Orden de activación: robot creado → broadcaster → controladores.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller, gripper_controller])),

        rviz,
    ])
