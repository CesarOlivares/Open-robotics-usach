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

"""Pruebas de joy_to_twist: zona muerta, curva expo y mapeo (evidencia D14)."""

from hexapod_control.joy_to_twist import (apply_deadzone, apply_expo,
                                          joy_to_velocity, shape_axis)
import pytest

P = {'deadzone': 0.08, 'expo': 0.5, 'v_max': 0.10, 'w_max': 0.45,
     'axis_x_sign': 1.0, 'axis_y_sign': 1.0}


def test_zona_muerta():
    """Bajo la zona muerta es 0; justo sobre ella parte de 0; el tope llega a 1."""
    assert apply_deadzone(0.05, 0.08) == 0.0
    assert apply_deadzone(-0.08, 0.08) == 0.0
    assert apply_deadzone(0.0801, 0.08) == pytest.approx(0.0, abs=1e-3)
    assert apply_deadzone(1.0, 0.08) == pytest.approx(1.0)
    assert apply_deadzone(-1.0, 0.08) == pytest.approx(-1.0)
    assert apply_deadzone(1.7, 0.08) == pytest.approx(1.0)  # lectura fuera de rango


def test_expo_suave_al_centro_y_completa_al_tope():
    """La expo deja 0 en 0, 1 en 1 y reduce la pendiente cerca del centro."""
    assert apply_expo(0.0, 0.5) == 0.0
    assert apply_expo(1.0, 0.5) == pytest.approx(1.0)
    assert apply_expo(-1.0, 0.5) == pytest.approx(-1.0)
    assert apply_expo(0.2, 0.5) < 0.2          # más fino que lineal al centro
    assert apply_expo(0.2, 0.0) == pytest.approx(0.2)


def test_gradiente_continuo_y_creciente():
    """Recorriendo la palanca de −1 a 1 la salida sube sin saltos (control suave)."""
    xs = [i / 1000 for i in range(-1000, 1001)]
    ys = [shape_axis(x, 0.08, 0.5) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))                 # monótona
    assert max(abs(b - a) for a, b in zip(ys, ys[1:])) < 0.003    # sin saltos


def test_mapeo_giro_y_lateral():
    """Y avanza; X gira (o va de lado en modo lateral); derecha = hacia −y."""
    vx, vy, wz = joy_to_velocity([0.0, 1.0], False, P)
    assert (vx, vy, wz) == pytest.approx((0.10, 0.0, 0.0))
    vx, vy, wz = joy_to_velocity([1.0, 0.0], False, P)
    assert (vx, vy, wz) == pytest.approx((0.0, 0.0, -0.45))
    vx, vy, wz = joy_to_velocity([1.0, 0.0], True, P)
    assert (vx, vy, wz) == pytest.approx((0.0, -0.10, 0.0))
    assert joy_to_velocity([0.03, -0.05], False, P) == (0.0, 0.0, 0.0)


def test_signo_de_ejes():
    """axis_x_sign = −1 invierte el sentido del eje X."""
    p = dict(P, axis_x_sign=-1.0)
    assert joy_to_velocity([1.0, 0.0], False, p)[2] == pytest.approx(0.45)


def test_ejes_faltantes():
    """Un mensaje con menos ejes no rompe el nodo: los que faltan valen 0."""
    assert joy_to_velocity([], False, P) == (0.0, 0.0, 0.0)
