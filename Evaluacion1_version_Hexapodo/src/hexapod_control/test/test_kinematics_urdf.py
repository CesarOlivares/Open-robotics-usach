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
La pata del URDF coincide con kinematics.fk (evidencia D4 del plan).

Se procesa urdf/leg_test.urdf.xacro, se encadenan las transformadas de sus
juntas (lo mismo que hace robot_state_publisher para publicar la TF) y la
posición del pie se compara con fk() para ángulos al azar.
"""

import math
import os
import random
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from hexapod_control.kinematics import fk, leg_geometry_from_urdf
import numpy as np
import pytest
import xacro

CHAIN = ['test_coxa_joint', 'test_femur_joint', 'test_tibia_joint', 'test_foot_joint']


@pytest.fixture(scope='module')
def urdf_xml():
    """URDF de la pata de prueba, ya procesado por xacro."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'leg_test.urdf.xacro')
    return xacro.process_file(path).toxml()


def rot_rpy(r, p, y):
    """Matriz de rotación de los ángulos roll-pitch-yaw del URDF (Rz·Ry·Rx)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    """Rotación de q rad en torno al eje unitario `axis` (Rodrigues)."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(q) * kx + (1 - math.cos(q)) * kx @ kx


def foot_from_urdf(urdf_xml, angles):
    """Posición del pie encadenando las juntas del URDF."""
    joints = {j.get('name'): j for j in ET.fromstring(urdf_xml).findall('joint')}
    q = dict(zip(CHAIN, list(angles) + [0.0]))
    T = np.eye(4)
    for name in CHAIN:
        j = joints[name]
        o = j.find('origin')
        xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
        step = np.eye(4)
        step[:3, :3] = rot_rpy(*rpy)
        step[:3, 3] = xyz
        T = T @ step
        if j.get('type') == 'revolute':
            axis = [float(v) for v in j.find('axis').get('xyz').split()]
            R = np.eye(4)
            R[:3, :3] = rot_axis(axis, q[name])
            T = T @ R
    return T[:3, 3]


def test_geometria_leida_del_urdf(urdf_xml):
    """Los largos que lee la cinemática son los de hexapod_params.xacro."""
    g = leg_geometry_from_urdf(urdf_xml, 'test')
    assert (g.coxa, g.femur, g.tibia) == pytest.approx((0.045, 0.080, 0.120))
    assert g.coxa_limits == pytest.approx((-math.pi / 4, math.pi / 4))


def test_urdf_coincide_con_fk(urdf_xml):
    """Para 500 poses al azar, pie del URDF = pie de fk() (error < 0,1 mm)."""
    g = leg_geometry_from_urdf(urdf_xml, 'test')
    rng = random.Random(3)
    for _ in range(500):
        angles = tuple(rng.uniform(lo, hi) for lo, hi in g.limits())
        err = np.linalg.norm(foot_from_urdf(urdf_xml, angles) - np.array(fk(g, angles)))
        assert err < 1e-4, f'{angles}: error {err * 1000:.4f} mm'
