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

"""Pruebas de la marcha trípode (sin ROS)."""

import math

from hexapod_control.gait import (GaitParams, GROUP_A, limit_command,
                                  step_vector, TripodGait)
import pytest

NAMES = ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')
ANGLES = (30, 90, 150, -30, -90, -150)
HOMES = [(0.215 * math.cos(math.radians(a)), 0.215 * math.sin(math.radians(a)), -0.09)
         for a in ANGLES]
DT = 0.02


def run(gait, seconds, v=(0.0, 0.0), wz=0.0):
    """Corre la marcha y devuelve la lista de posiciones de pies en cada paso."""
    out = []
    for _ in range(round(seconds / DT)):
        out.append(gait.update(DT, v, wz))
    return out


def test_quieto_sin_comando():
    """Sin velocidad pedida los pies no se mueven y la fase no avanza."""
    g = TripodGait(NAMES, HOMES)
    feet = run(g, 2.0)[-1]
    assert feet == pytest.approx(HOMES)
    assert g.phase == 0.0


def test_siempre_tres_patas_apoyadas_y_alternadas():
    """En todo instante apoya exactamente un grupo (3 patas)."""
    g = TripodGait(NAMES, HOMES)
    for _ in range(200):
        g.update(DT, (0.08, 0.0), 0.0)
        apoyadas = {n for i, n in enumerate(NAMES) if g.in_stance(i)}
        assert len(apoyadas) == 3
        assert apoyadas == set(GROUP_A) or apoyadas.isdisjoint(GROUP_A)


def test_pies_apoyados_a_la_altura_del_suelo():
    """Un pie apoyado está siempre a la altura de su posición neutra."""
    g = TripodGait(NAMES, HOMES)
    for _ in range(200):
        feet = g.update(DT, (0.08, 0.02), 0.2)
        for i, f in enumerate(feet):
            if g.in_stance(i):
                assert f[2] == pytest.approx(HOMES[i][2])


def test_pie_apoyado_se_mueve_al_reves_del_cuerpo():
    """Al avanzar en +x, un pie apoyado retrocede a velocidad −v respecto del cuerpo."""
    g = TripodGait(NAMES, HOMES)
    run(g, 2.0, (0.08, 0.0))
    i = next(i for i in range(6) if g.in_stance(i) and g.leg_phase(i) < 0.3)
    x0 = g.feet[i][0]
    g.update(DT, (0.08, 0.0), 0.0)
    assert g.feet[i][0] - x0 == pytest.approx(-0.08 * DT)


def test_igual_a_las_formulas_del_plan():
    """Con velocidad constante, tras un ciclo el pie sigue las fórmulas cerradas."""
    p = GaitParams()
    g = TripodGait(NAMES, HOMES, p)
    v = (0.08, -0.03)
    run(g, 2.0 * p.period, v)
    for _ in range(50):
        feet = g.update(DT, v, 0.0)
        for i, f in enumerate(feet):
            phi = g.leg_phase(i)
            d = step_vector(p, v, 0.0, HOMES[i])
            if phi < p.duty:
                k = 0.5 - phi / p.duty
                expected = (HOMES[i][0] + k * d[0], HOMES[i][1] + k * d[1], HOMES[i][2])
            else:
                u = (phi - p.duty) / (1 - p.duty)
                expected = (HOMES[i][0] + (u - 0.5) * d[0], HOMES[i][1] + (u - 0.5) * d[1],
                            HOMES[i][2] + p.step_height * math.sin(math.pi * u))
            assert f == pytest.approx(expected, abs=1e-6)


def test_al_soltar_vuelve_a_casa_y_se_detiene():
    """Tras soltar el mando, en menos de un ciclo los pies vuelven a la postura neutra."""
    g = TripodGait(NAMES, HOMES)
    run(g, 1.3, (0.08, 0.0), 0.3)
    run(g, 1.2 * g.params.period)
    assert g.at_home()
    assert not g.walking
    fase = g.phase
    run(g, 0.5)
    assert g.phase == fase


def test_sin_saltos_al_cambiar_la_velocidad():
    """Si la velocidad cambia de golpe, ningún pie salta más que un paso normal."""
    g = TripodGait(NAMES, HOMES)
    prev = run(g, 1.27, (0.08, 0.0))[-1]
    for cmd in [((-0.08, 0.0), 0.0), ((0.0, 0.08), 0.4), ((0.0, 0.0), 0.0)]:
        for _ in range(30):
            feet = g.update(DT, *cmd)
            for a, b in zip(prev, feet):
                assert math.dist(a, b) < 0.012
            prev = feet


def test_limite_de_paso():
    """Una velocidad excesiva se escala para que ningún paso supere max_step."""
    p = GaitParams()
    v, wz, k = limit_command(p, (1.0, 0.0), 1.0, HOMES)
    assert k < 1.0
    assert max(math.hypot(*step_vector(p, v, wz, h)) for h in HOMES) == pytest.approx(p.max_step)
    assert v[0] / wz == pytest.approx(1.0)  # misma dirección


def test_giro_en_el_lugar_mueve_los_pies_tangencialmente():
    """Con solo ω, el paso de cada pata es perpendicular a su radio."""
    p = GaitParams()
    for h in HOMES:
        d = step_vector(p, (0.0, 0.0), 0.3, h)
        assert d[0] * h[0] + d[1] * h[1] == pytest.approx(0.0, abs=1e-12)


def test_teclado():
    """Las teclas suben y bajan la velocidad, con tope y freno."""
    from hexapod_control.keyboard_teleop import apply_key, V_MAX
    cmd = (0.0, 0.0, 0.0)
    cmd = apply_key(cmd, 'w')
    assert cmd == (0.02, 0.0, 0.0)
    for _ in range(20):
        cmd = apply_key(cmd, 'w')
    assert cmd[0] == V_MAX
    assert apply_key(cmd, 'q')[2] == pytest.approx(0.1)
    assert apply_key(cmd, ' ') == (0.0, 0.0, 0.0)
    assert apply_key(cmd, 'x') is None
