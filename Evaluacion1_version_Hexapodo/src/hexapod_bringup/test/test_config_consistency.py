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

Los nombres de articulaciones y tópicos aparecen en varios archivos (URDF,
controllers.yaml, gz_bridge.yaml, gait_node). Si uno se escribe distinto, el
error solo aparecería al ejecutar. Estos tests lo detectan con colcon test,
sin abrir Gazebo.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from hexapod_control.robot_model import HexapodModel
import pytest
import xacro
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), '..', 'config')


def load_yaml(name):
    """Carga un YAML de config/."""
    with open(os.path.join(CONFIG_DIR, name)) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def urdf_xml():
    """URDF del hexápodo tal como lo procesa el launch de Gazebo."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    return xacro.process_file(
        path, mappings={'controllers_file': '/tmp/controllers.yaml'}).toxml()


def test_legs_controller_igual_a_gait_node(urdf_xml):
    """legs_controller recibe las 18 juntas en el mismo orden que publica gait_node."""
    joints = load_yaml('controllers.yaml')['legs_controller']['ros__parameters']['joints']
    assert joints == HexapodModel(urdf_xml).joint_names()


def test_juntas_comandables_en_ros2_control(urdf_xml):
    """Cada junta del controlador declara interfaz de comando de posición y effort."""
    root = ET.fromstring(urdf_xml)
    block = root.find('ros2_control')
    declared = {j.get('name'): j for j in block.findall('joint')}
    joints = load_yaml('controllers.yaml')['legs_controller']['ros__parameters']['joints']
    for name in joints:
        j = declared[name]
        assert j.find("command_interface[@name='position']") is not None
        assert j.find("state_interface[@name='effort']") is not None


def test_plugin_recibe_el_yaml(urdf_xml):
    """El plugin gz_ros2_control recibe la ruta de controllers.yaml."""
    params = ET.fromstring(urdf_xml).find('.//plugin[@filename="gz_ros2_control-system"]')
    assert params.find('parameters').text == '/tmp/controllers.yaml'


def test_bridge_tiene_reloj_y_odometria():
    """El puente pasa /clock, la odometría y la TF odom → base_link."""
    ros_topics = {e['ros_topic_name'] for e in load_yaml('gz_bridge.yaml')}
    assert {'/clock', '/hexapod/odom', '/tf'} <= ros_topics


def test_posturas_de_gait_yaml_alcanzables(urdf_xml):
    """Las posturas recogida y de pie de gait.yaml respetan los límites articulares."""
    p = load_yaml('gait.yaml')['gait_node']['ros__parameters']
    model = HexapodModel(urdf_xml)
    l1 = model.legs[0].geom.coxa
    for radial, height in ((p['stand_radial'], p['stand_height']),
                           (p['tuck_radial'], p['tuck_height'])):
        model.joint_angles(model.neutral_feet(l1 + radial, height))  # lanza si no


def test_tope_del_teclado_igual_al_de_la_marcha():
    """La velocidad máxima del teclado es la que la marcha puede dar: max_step / (β T)."""
    from hexapod_control.keyboard_teleop import V_MAX
    p = load_yaml('gait.yaml')['gait_node']['ros__parameters']
    assert V_MAX == pytest.approx(p['max_step'] / (p['duty'] * p['period']))
