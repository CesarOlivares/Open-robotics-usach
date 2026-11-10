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
Consistencia entre el URDF y los archivos de configuración.

Los nombres de articulaciones y tópicos aparecen en varios archivos
(URDF, controllers.yaml, teleop.yaml, gz_bridge.yaml). Si uno se escribe
distinto, el error solo aparecería al ejecutar. Estos tests lo detectan
con colcon test, sin abrir Gazebo.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from octavius_bringup.arm_logic import parse_joint_limits
import pytest
import xacro
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def load_yaml(name):
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def urdf_root():
    xacro_file = os.path.join(get_package_share_directory('octavius_description'),
                              'urdf', 'octavius.urdf.xacro')
    doc = xacro.process_file(xacro_file,
                             mappings={'controllers_file': '/tmp/controllers.yaml'})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope='module')
def limits(urdf_root):
    return parse_joint_limits(ET.tostring(urdf_root, encoding='unicode'))


@pytest.fixture(scope='module')
def controllers():
    return load_yaml('controllers.yaml')


@pytest.fixture(scope='module')
def teleop():
    return load_yaml('teleop.yaml')['arm_commander']['ros__parameters']


def ros2_control_command_joints(urdf_root):
    """Juntas que declaran una interfaz de comando en <ros2_control>."""
    return {j.get('name') for j in urdf_root.iter('joint')
            if j.find('command_interface') is not None}


def test_robot_has_six_degrees_of_freedom(limits, teleop):
    # 5 articulaciones del brazo + la garra (2 juntas, un solo GDL)
    assert len(limits) == len(teleop['arm_joints']) + len(teleop['gripper_joints'])
    assert len(teleop['arm_joints']) + 1 == 6


def test_controller_joints_exist_and_are_commandable(urdf_root, limits,
                                                     controllers):
    commandable = ros2_control_command_joints(urdf_root)
    for name in ('arm_controller', 'gripper_controller'):
        for joint in controllers[name]['ros__parameters']['joints']:
            assert joint in limits, f'{name}: {joint} no existe en el URDF'
            assert joint in commandable, \
                f'{name}: {joint} no tiene command_interface en <ros2_control>'


def test_teleop_arm_joints_match_controller_order(controllers, teleop):
    # arm_commander publica los ángulos en este orden: debe ser el mismo
    # que espera arm_controller.
    assert teleop['arm_joints'] == \
        controllers['arm_controller']['ros__parameters']['joints']


def test_teleop_input_joints_are_arm_joints(teleop):
    for joint in teleop['joystick_joints'] + teleop['keyboard_joints']:
        assert joint in teleop['arm_joints']
    assert len(teleop['joystick_axis_sign']) == len(teleop['joystick_joints'])


def test_limit_margin_fits_every_arm_joint(limits, teleop):
    for joint in teleop['arm_joints']:
        lower, upper = limits[joint]
        assert 2 * teleop['limit_margin'] < upper - lower


def test_gripper_joints_match_controller(controllers, teleop):
    assert teleop['gripper_joints'] == \
        controllers['gripper_controller']['ros__parameters']['joints']


def test_gripper_values_within_limits(limits, teleop):
    for joint in teleop['gripper_joints']:
        lower, upper = limits[joint]
        assert lower <= teleop['gripper_closed'] <= upper
        assert lower <= teleop['gripper_open'] <= upper


def test_bridge_covers_every_sensor_topic(urdf_root):
    bridged = {entry['gz_topic_name'] for entry in load_yaml('gz_bridge.yaml')}
    for sensor in urdf_root.iter('sensor'):
        topic = '/' + sensor.find('topic').text.lstrip('/')
        assert topic in bridged, f'sensor {sensor.get("name")}: {topic} sin bridge'
    assert '/clock' in bridged
