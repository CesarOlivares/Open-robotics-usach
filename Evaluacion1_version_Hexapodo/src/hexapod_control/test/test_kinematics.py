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

"""Pruebas de la cinemática directa e inversa de una pata (sin ROS)."""

import math
import random

from hexapod_control.kinematics import fk, ik, LegGeometry, UnreachableError
import pytest

GEOM = LegGeometry(coxa=0.045, femur=0.080, tibia=0.120,
                   coxa_limits=(-math.pi / 4, math.pi / 4),
                   femur_limits=(-math.pi / 2, math.pi / 2),
                   tibia_limits=(-2.5, 0.6))
TOL = 1e-4  # 0,1 mm


def dist(a, b):
    """Distancia euclidiana entre dos puntos 3D."""
    return math.dist(a, b)


def test_pata_recta():
    """Con todo en 0 la pata es una recta horizontal de largo L1 + L2 + L3."""
    assert dist(fk(GEOM, (0.0, 0.0, 0.0)), (0.245, 0.0, 0.0)) < 1e-12


def test_casos_a_mano():
    """Poses que se pueden calcular sin la fórmula general."""
    # Fémur vertical hacia arriba y tibia doblada 90° (horizontal hacia afuera)
    p = fk(GEOM, (0.0, math.pi / 2, -math.pi / 2))
    assert dist(p, (0.045 + 0.120, 0.0, 0.080)) < 1e-12
    # Coxa girada 90°: la pata recta apunta a +y
    p = fk(GEOM, (math.pi / 2, 0.0, 0.0))
    assert dist(p, (0.0, 0.245, 0.0)) < 1e-12


def test_ik_fk_ida_y_vuelta():
    """FK(IK(p)) = p para miles de puntos alcanzables (error < 0,1 mm)."""
    rng = random.Random(42)
    peor = 0.0
    n = 0
    while n < 5000:
        angles = tuple(rng.uniform(lo, hi) for lo, hi in GEOM.limits())
        p = fk(GEOM, angles)
        try:
            sol = ik(GEOM, p)
        except UnreachableError:
            # La solución rodilla-arriba puede salir de los límites aunque la
            # rodilla-abajo esté dentro: ese punto no cuenta para la prueba.
            continue
        peor = max(peor, dist(fk(GEOM, sol), p))
        n += 1
    assert peor < TOL, f'error máximo {peor * 1000:.4f} mm'


def test_ik_devuelve_los_mismos_angulos_con_rodilla_arriba():
    """
    Si los ángulos de partida tienen θ3 < 0, la IK los recupera.

    Solo vale con el pie hacia afuera del eje de la coxa (r > 0). Si el pie
    queda detrás del eje, el mismo punto se alcanza con la coxa girada 180°;
    esa zona está bajo el cuerpo y la marcha nunca la usa.
    """
    rng = random.Random(7)
    n = 0
    while n < 2000:
        angles = (rng.uniform(-0.7, 0.7), rng.uniform(-1.4, 1.4), rng.uniform(-2.4, -0.05))
        x, y, _ = fk(GEOM, angles)
        if math.cos(angles[0]) * x + math.sin(angles[0]) * y <= 0.0:
            continue
        n += 1
        sol = ik(GEOM, fk(GEOM, angles), check_limits=False)
        assert max(abs(a - b) for a, b in zip(angles, sol)) < 1e-6


def test_postura_de_pie():
    """La postura de partida (pie a 7 cm del fémur, 9 cm bajo él) es alcanzable."""
    p = (0.045 + 0.070, 0.0, -0.090)
    t1, t2, t3 = ik(GEOM, p)
    assert abs(t1) < 1e-12
    assert t3 < 0.0
    assert dist(fk(GEOM, (t1, t2, t3)), p) < TOL


@pytest.mark.parametrize('p', [
    (0.40, 0.0, 0.0),          # más lejos que L2 + L3
    (0.045 + 0.01, 0.0, 0.0),  # más cerca que |L2 − L3|
])
def test_fuera_de_alcance(p):
    """Un punto inalcanzable lanza UnreachableError, no ángulos inválidos."""
    with pytest.raises(UnreachableError):
        ik(GEOM, p)


def test_fuera_de_limites():
    """Un punto alcanzable pero que exige girar la coxa 90° se rechaza."""
    with pytest.raises(UnreachableError):
        ik(GEOM, (0.0, 0.15, -0.05))
    ik(GEOM, (0.0, 0.15, -0.05), check_limits=False)  # sin límites, sí tiene solución
