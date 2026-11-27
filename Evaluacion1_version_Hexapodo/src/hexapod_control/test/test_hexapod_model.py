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
Revisión del hexápodo completo (evidencias D5–D7 del plan).

Se procesa urdf/hexapod.urdf.xacro y se comprueba:
    - 18 articulaciones revolutas (3 por pata) y las 6 patas en sus vértices;
    - masa total = la estimada en el plan (1,87 ± 0,05 kg);
    - centro de masa en el centro del cuerpo (el layout está equilibrado);
    - cada pata tiene la misma geometría que lee la cinemática.
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from hexapod_control.kinematics import fk, leg_geometry_from_urdf
import numpy as np
import pytest
import xacro

LEGS = {'lf': 30, 'lm': 90, 'lr': 150, 'rf': -30, 'rm': -90, 'rr': -150}
BODY_RADIUS = 0.100


@pytest.fixture(scope='module')
def root():
    """Árbol XML del hexápodo, ya procesado por xacro."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    return ET.fromstring(xacro.process_file(path).toxml())


def rot_rpy(r, p, y):
    """Matriz de rotación roll-pitch-yaw del URDF (Rz·Ry·Rx)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def origin_tf(elem):
    """Transformada 4×4 del <origin> de una junta o de un <inertial>."""
    T = np.eye(4)
    o = elem.find('origin') if elem is not None else None
    if o is not None:
        T[:3, :3] = rot_rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def link_poses(root):
    """Pose de cada link respecto a base_link con todas las juntas en 0."""
    poses = {'base_link': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        rest = []
        for j in pending:
            parent = j.find('parent').get('link')
            if parent in poses:
                poses[j.find('child').get('link')] = poses[parent] @ origin_tf(j)
            else:
                rest.append(j)
        assert len(rest) < len(pending), 'el árbol tiene links sueltos'
        pending = rest
    return poses


def test_18_articulaciones(root):
    """Hay exactamente 3 revolutas por pata, con los nombres esperados."""
    rev = sorted(j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute')
    esperadas = sorted(f'{p}_{s}_joint' for p in LEGS for s in ('coxa', 'femur', 'tibia'))
    assert rev == esperadas


def test_patas_en_los_vertices(root):
    """Cada coxa está a R del centro, en su ángulo, con la pata mirando hacia afuera."""
    joints = {j.get('name'): j for j in root.findall('joint')}
    for prefix, deg in LEGS.items():
        T = origin_tf(joints[f'{prefix}_coxa_joint'])
        phi = math.radians(deg)
        assert T[:3, 3] == pytest.approx([BODY_RADIUS * math.cos(phi),
                                          BODY_RADIUS * math.sin(phi), 0.0], abs=1e-9)
        assert T[:3, 0] == pytest.approx([math.cos(phi), math.sin(phi), 0.0], abs=1e-9)


def test_masa_total(root):
    """La masa del modelo es la del presupuesto del plan: 1,87 ± 0,05 kg."""
    total = sum(float(link.find('inertial/mass').get('value'))
                for link in root.findall('link') if link.find('inertial') is not None)
    assert total == pytest.approx(1.87, abs=0.05)


def test_centro_de_masa_centrado(root):
    """Con las patas rectas, el centro de masa queda a menos de 5 mm del eje z."""
    poses = link_poses(root)
    m_total, c = 0.0, np.zeros(3)
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        m = float(inertial.find('mass').get('value'))
        c += m * (poses[link.get('name')] @ origin_tf(inertial))[:3, 3]
        m_total += m
    c /= m_total
    assert math.hypot(c[0], c[1]) < 0.005, f'CM en {c}'


@pytest.mark.parametrize('prefix', list(LEGS))
def test_cada_pata_igual_a_la_cinematica(root, prefix):
    """El pie de cada pata (patas rectas) está donde dice fk() + montaje."""
    urdf_xml = ET.tostring(root, encoding='unicode')
    g = leg_geometry_from_urdf(urdf_xml, prefix)
    assert (g.coxa, g.femur, g.tibia) == pytest.approx((0.045, 0.080, 0.120))
    foot_local = np.array(list(fk(g, (0.0, 0.0, 0.0))) + [1.0])
    coxa = link_poses(root)[f'{prefix}_coxa_link']
    phi = math.radians(LEGS[prefix])
    reach = BODY_RADIUS + g.coxa + g.femur + g.tibia
    assert (coxa @ foot_local)[:3] == pytest.approx(
        [reach * math.cos(phi), reach * math.sin(phi), 0.0], abs=1e-9)
