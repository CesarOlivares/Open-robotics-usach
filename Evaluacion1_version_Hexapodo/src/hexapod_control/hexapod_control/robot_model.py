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
Modelo del hexápodo completo, leído del URDF (sin ROS).

Junta lo que necesita la marcha: dónde está montada cada pata en el cuerpo
y su geometría. Así los nodos no repiten números de hexapod_params.xacro.

Marcos:
    cuerpo  base_link: x adelante, y a la izquierda, z arriba.
    pata    marco de su coxa: origen en el eje de la coxa, x hacia afuera.
            Se obtiene del cuerpo girando φ (ángulo de montaje) en torno a z
            y trasladando a (R cos φ, R sin φ, 0).
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

from hexapod_control.kinematics import ik, leg_geometry_from_urdf, LegGeometry

# Orden de las patas en todo el proyecto (y en legs_controller):
# izquierda adelante, medio, atrás; derecha adelante, medio, atrás.
LEG_NAMES = ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')
SEGMENTS = ('coxa', 'femur', 'tibia')


@dataclass(frozen=True)
class LegMount:
    """Una pata: nombre, posición de la coxa en el cuerpo, ángulo y geometría."""

    name: str
    x: float
    y: float
    angle: float
    geom: LegGeometry

    def body_to_leg(self, p):
        """Punto del marco del cuerpo → marco de la pata."""
        dx, dy = p[0] - self.x, p[1] - self.y
        c, s = math.cos(self.angle), math.sin(self.angle)
        return (c * dx + s * dy, -s * dx + c * dy, p[2])

    def leg_to_body(self, p):
        """Punto del marco de la pata → marco del cuerpo."""
        c, s = math.cos(self.angle), math.sin(self.angle)
        return (self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1], p[2])


class HexapodModel:
    """Las 6 patas del robot, en el orden de LEG_NAMES."""

    def __init__(self, urdf_xml):
        """Lee montaje y geometría de cada pata desde el URDF en texto."""
        joints = {j.get('name'): j for j in ET.fromstring(urdf_xml).findall('joint')}
        legs = []
        for name in LEG_NAMES:
            origin = joints[f'{name}_coxa_joint'].find('origin')
            x, y, _ = (float(v) for v in origin.get('xyz').split())
            yaw = float(origin.get('rpy', '0 0 0').split()[2])
            legs.append(LegMount(name, x, y, yaw, leg_geometry_from_urdf(urdf_xml, name)))
        self.legs = tuple(legs)

    @staticmethod
    def joint_names():
        """Las 18 articulaciones en el orden de legs_controller."""
        return [f'{leg}_{seg}_joint' for leg in LEG_NAMES for seg in SEGMENTS]

    def neutral_feet(self, radial, height):
        """
        Posición neutra de cada pie en el marco del cuerpo.

        radial  distancia horizontal del eje de la coxa al pie [m]
        height  altura del cuerpo (eje de la coxa) sobre el centro del pie [m]
        Cada pie queda en la dirección de su pata, a `height` bajo el cuerpo.
        """
        return [leg.leg_to_body((radial, 0.0, -height)) for leg in self.legs]

    def joint_angles(self, feet_body, check_limits=True):
        """IK de las 6 patas: pies en el marco del cuerpo → 18 ángulos."""
        angles = []
        for leg, p in zip(self.legs, feet_body):
            angles.extend(ik(leg.geom, leg.body_to_leg(p), check_limits))
        return angles
