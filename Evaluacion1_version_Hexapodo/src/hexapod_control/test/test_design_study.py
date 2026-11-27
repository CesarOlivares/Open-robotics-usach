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

"""Pruebas del estudio de dimensiones (E8)."""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from hexapod_control.design_study import (Components, Design, evaluate, foot_load,
                                          hexagon_contains, joint_torques,
                                          min_body_radius, min_coxa_length,
                                          torque_safety)
import pytest


def read_params():
    """Valores numéricos de hexapod_params.xacro."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod_params.xacro')
    out = {}
    for prop in ET.parse(path).getroot():
        try:
            out[prop.get('name')] = float(prop.get('value'))
        except (TypeError, ValueError):
            pass
    return out


def test_modelo_de_torque_igual_a_gazebo():
    """Con la geometría de partida, el modelo da lo que midió Gazebo en E4 (3 patas)."""
    f, t = joint_torques(Design(0.045, 0.080, 0.120, 0.100, 0.070, 0.090), 0.070, -0.090)
    assert f == pytest.approx(0.375, abs=0.003)
    assert t == pytest.approx(0.025, abs=0.003)


def test_carga_por_pie():
    """F = M g / 3 con la masa del modelo."""
    assert foot_load(Components()) == pytest.approx(1.876 * 9.81 / 3)


def test_hexagono():
    """El hexágono tiene el lado plano al frente: (apotema, 0) está en el borde."""
    assert hexagon_contains(0.1, 0.0999, 0.0)
    assert not hexagon_contains(0.1, 0.1001, 0.0)
    assert hexagon_contains(0.1, 0.0, 0.1 / math.cos(math.radians(30)) - 1e-4)


def test_empaque():
    """L1 y R mínimos salen de las medidas reales (servo, placa, batería, PCA)."""
    assert min_coxa_length() == pytest.approx(0.015 + 0.0407 / 2 + 0.005)
    assert 0.075 < min_body_radius() < 0.085


def test_diseno_de_partida_no_cumple():
    """El diseño de E0–E7 pasa el torque de diseño del fémur y le falta altura libre."""
    r = evaluate(Design(0.045, 0.080, 0.120, 0.100, 0.070, 0.090))
    assert not r['torque fémur [N·m]'][2]
    assert not r['altura libre [m]'][2]


def test_modelo_actual_cumple_todo():
    """Las medidas de hexapod_params.xacro y la postura de gait.yaml cumplen todo."""
    p = read_params()
    d = Design(p['coxa_length'], p['femur_length'], p['tibia_length'], p['body_radius'],
               0.060, 0.100)
    r = evaluate(d)
    fallas = [k for k, v in r.items() if k != 'feasible' and not v[2]]
    assert r['feasible'], f'no cumple: {fallas}'
    assert torque_safety(r) >= 2.0
