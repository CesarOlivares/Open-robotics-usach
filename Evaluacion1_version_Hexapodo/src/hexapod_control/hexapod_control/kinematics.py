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
Cinemática de una pata de 3 articulaciones (coxa, fémur, tibia).

No depende de ROS: se puede probar y usar en el estudio de dimensiones.

Marco de la pata (el de la articulación de la coxa, igual que en leg.xacro):
    x hacia afuera del cuerpo, z hacia arriba, origen en el eje de la coxa.

Ángulos (rad):
    θ1 coxa   giro en torno a z (positivo = antihorario visto desde arriba)
    θ2 fémur  positivo sube el fémur
    θ3 tibia  ángulo de la tibia respecto del fémur; negativo la dobla hacia abajo

Con los tres ángulos en 0 la pata queda recta y horizontal: el pie está en
(L1 + L2 + L3, 0, 0).

Cinemática directa (FK):
    r = L1 + L2 cos θ2 + L3 cos(θ2 + θ3)
    x = r cos θ1,  y = r sin θ1,  z = L2 sin θ2 + L3 sin(θ2 + θ3)

Cinemática inversa (IK), solución "rodilla hacia arriba":
    θ1 = atan2(y, x)
    ρ  = √(x² + y²) − L1         distancia horizontal del eje del fémur al pie
    D² = ρ² + z²                 distancia (al cuadrado) del eje del fémur al pie
    cos θ3 = (D² − L2² − L3²) / (2 L2 L3)        ley de cosenos
    θ3 = −arccos(cos θ3)
    θ2 = atan2(z, ρ) − atan2(L3 sin θ3, L2 + L3 cos θ3)
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


class UnreachableError(ValueError):
    """El punto pedido está fuera del alcance o de los límites de la pata."""


@dataclass(frozen=True)
class LegGeometry:
    """Largos [m] y límites articulares [rad] de una pata."""

    coxa: float
    femur: float
    tibia: float
    coxa_limits: tuple = (-math.pi, math.pi)
    femur_limits: tuple = (-math.pi, math.pi)
    tibia_limits: tuple = (-math.pi, math.pi)

    def limits(self):
        """Devuelve los límites en orden (coxa, fémur, tibia)."""
        return (self.coxa_limits, self.femur_limits, self.tibia_limits)


def fk(geom, angles):
    """
    Cinemática directa: ángulos (θ1, θ2, θ3) → posición del pie (x, y, z).

    La posición está en el marco de la pata, en metros.
    """
    t1, t2, t3 = angles
    r = geom.coxa + geom.femur * math.cos(t2) + geom.tibia * math.cos(t2 + t3)
    z = geom.femur * math.sin(t2) + geom.tibia * math.sin(t2 + t3)
    return (r * math.cos(t1), r * math.sin(t1), z)


def ik(geom, point, check_limits=True):
    """
    Cinemática inversa: posición del pie (x, y, z) → ángulos (θ1, θ2, θ3).

    Devuelve la solución con la rodilla hacia arriba (θ3 ≤ 0) y con el pie
    hacia afuera del eje de la coxa: θ1 apunta hacia el pie. Un pie detrás
    del eje (bajo el cuerpo) también se alcanzaría con θ1 + π, pero la
    marcha nunca usa esa zona.
    Lanza UnreachableError si el punto está fuera del alcance o, con
    check_limits=True, si algún ángulo sale de sus límites.
    """
    x, y, z = point
    t1 = math.atan2(y, x)
    rho = math.hypot(x, y) - geom.coxa
    d2 = rho * rho + z * z
    l2, l3 = geom.femur, geom.tibia

    cos_t3 = (d2 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    # |cos θ3| > 1 significa que no existe el triángulo fémur–tibia–pie:
    # el pie está más lejos que L2 + L3 o más cerca que |L2 − L3|.
    if cos_t3 > 1.0 + 1e-9 or cos_t3 < -1.0 - 1e-9:
        raise UnreachableError(
            f'punto fuera de alcance: distancia {math.sqrt(d2):.4f} m, '
            f'rango [{abs(l2 - l3):.4f}, {l2 + l3:.4f}] m')
    cos_t3 = max(-1.0, min(1.0, cos_t3))
    t3 = -math.acos(cos_t3)
    t2 = math.atan2(z, rho) - math.atan2(l3 * math.sin(t3), l2 + l3 * math.cos(t3))

    angles = (t1, t2, t3)
    if check_limits:
        names = ('coxa', 'fémur', 'tibia')
        for name, value, (lo, hi) in zip(names, angles, geom.limits()):
            if not lo - 1e-9 <= value <= hi + 1e-9:
                raise UnreachableError(
                    f'{name} = {value:.3f} rad fuera de [{lo:.3f}, {hi:.3f}]')
    return angles


def leg_geometry_from_urdf(urdf_xml, prefix):
    """
    Lee los largos y límites de la pata `prefix` desde el URDF.

    Así la cinemática usa exactamente los números de hexapod_params.xacro:
        L1 = x del origen de {prefix}_femur_joint
        L2 = x del origen de {prefix}_tibia_joint
        L3 = x del origen de {prefix}_foot_joint
    """
    root = ET.fromstring(urdf_xml)
    joints = {j.get('name'): j for j in root.findall('joint')}

    def origin_x(name):
        return float(joints[name].find('origin').get('xyz').split()[0])

    def limits(name):
        lim = joints[name].find('limit')
        return (float(lim.get('lower')), float(lim.get('upper')))

    return LegGeometry(
        coxa=origin_x(f'{prefix}_femur_joint'),
        femur=origin_x(f'{prefix}_tibia_joint'),
        tibia=origin_x(f'{prefix}_foot_joint'),
        coxa_limits=limits(f'{prefix}_coxa_joint'),
        femur_limits=limits(f'{prefix}_femur_joint'),
        tibia_limits=limits(f'{prefix}_tibia_joint'),
    )
