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

"""Pruebas de robot_model: montaje de las patas y posturas de gait_node."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from hexapod_control.kinematics import fk
from hexapod_control.robot_model import HexapodModel, LEG_NAMES
import pytest
import xacro


@pytest.fixture(scope='module')
def model():
    """Modelo leído del hexápodo completo."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    return HexapodModel(xacro.process_file(path).toxml())


def test_orden_de_patas_y_juntas(model):
    """Las patas y las 18 juntas siguen el orden de legs_controller."""
    assert tuple(leg.name for leg in model.legs) == LEG_NAMES
    names = model.joint_names()
    assert len(names) == 18
    assert names[:3] == ['lf_coxa_joint', 'lf_femur_joint', 'lf_tibia_joint']


def test_cambio_de_marco_ida_y_vuelta(model):
    """body_to_leg y leg_to_body son inversas."""
    p = (0.12, -0.07, -0.05)
    for leg in model.legs:
        assert leg.leg_to_body(leg.body_to_leg(p)) == pytest.approx(p, abs=1e-12)


def test_pie_neutro_en_la_direccion_de_su_pata(model):
    """El pie neutro queda en la dirección del vértice, a R + radial."""
    for leg, p in zip(model.legs, model.neutral_feet(0.115, 0.09)):
        assert math.atan2(p[1], p[0]) == pytest.approx(leg.angle, abs=1e-9)
        assert math.hypot(p[0], p[1]) == pytest.approx(0.100 + 0.115, abs=1e-9)
        assert p[2] == pytest.approx(-0.09)


@pytest.mark.parametrize('radial, height', [(0.045 + 0.070, 0.090),   # de pie
                                            (0.045 + 0.080, 0.030)])  # recogida
def test_posturas_de_gait_node(model, radial, height):
    """Las posturas de gait.yaml respetan los límites y la FK devuelve el pie."""
    feet = model.neutral_feet(radial, height)
    angles = model.joint_angles(feet)
    for i, leg in enumerate(model.legs):
        p = leg.leg_to_body(fk(leg.geom, angles[3 * i:3 * i + 3]))
        assert p == pytest.approx(feet[i], abs=1e-9)
