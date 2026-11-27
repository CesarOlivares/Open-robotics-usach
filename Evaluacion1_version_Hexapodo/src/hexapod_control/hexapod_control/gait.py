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
Marcha trípode (sin ROS): de la velocidad pedida a la posición de cada pie.

Grupos que se alternan (siempre hay un triángulo de 3 patas en el suelo):
    A = lf, rm, lr   (fase 0)
    B = rf, lm, rr   (desfasado medio ciclo)

Cada pata tiene una fase φ ∈ [0, 1) que avanza dt/T por paso. Con fracción
de apoyo β = 0,5:
    apoyo (φ < β)   el pie está en el suelo y se mueve respecto del cuerpo
                    al revés de cómo se mueve el cuerpo: −(v + ω ẑ × p)·dt.
                    Eso empuja al robot.
    vuelo (φ ≥ β)   el pie viaja desde donde despegó hasta p0 + Δ/2, por un
                    arco de altura h:  u = (φ − β)/(1 − β),
                    p = p_despegue + u (p_destino − p_despegue) + h sin(πu) ẑ

Vector de paso (lo que avanza el pie en un apoyo), con giro incluido:
    Δ = β T (v + ω ẑ × p0)

Con velocidad constante esto es igual a las fórmulas cerradas del plan:
    apoyo  p = p0 + (1/2 − φ/β) Δ
    vuelo  p = p0 + (u − 1/2) Δ + h sin(πu) ẑ
Se calcula de forma incremental para que, si la velocidad cambia a mitad de
paso, el pie no salte: el apoyo sigue desde donde está.

Al soltar el mando (velocidad 0) la marcha sigue hasta que todos los pies
vuelven a su posición neutra y ahí se detiene.

Todas las posiciones están en el marco del cuerpo (base_link).
"""

from dataclasses import dataclass
import math

GROUP_A = ('lf', 'rm', 'lr')
GROUP_B = ('rf', 'lm', 'rr')


@dataclass
class GaitParams:
    """Parámetros de la marcha."""

    period: float = 1.0        # T: duración de un ciclo completo [s]
    duty: float = 0.5          # β: fracción del ciclo en apoyo
    step_height: float = 0.03  # h: altura del arco de vuelo [m]
    max_step: float = 0.06     # |Δ| máximo en cualquier pata [m] (alcance)
    home_tol: float = 1e-4     # [m] un pie a menos de esto está "en casa"
    max_accel: float = 0.4     # [m/s²] cambio máximo de la velocidad lineal
    max_alpha: float = 2.0     # [rad/s²] cambio máximo de la velocidad de giro


def step_vector(params, v, wz, p0):
    """Δ = β T (v + ω ẑ × p0), solo en el plano xy."""
    bt = params.duty * params.period
    return (bt * (v[0] - wz * p0[1]), bt * (v[1] + wz * p0[0]))


def limit_command(params, v, wz, homes):
    """
    Escala (v, ω) para que ningún Δ supere max_step.

    Devuelve el comando escalado y el factor usado (1 si no hizo falta).
    Se escalan juntos para no cambiar la dirección del movimiento.
    """
    worst = max(math.hypot(*step_vector(params, v, wz, p0)) for p0 in homes)
    k = 1.0 if worst <= params.max_step else params.max_step / worst
    return (v[0] * k, v[1] * k), wz * k, k


class TripodGait:
    """Estado de la marcha: fase global y posición actual de cada pie."""

    def __init__(self, leg_names, homes, params=None):
        """
        leg_names y homes en el mismo orden.

        homes: posición neutra de cada pie en el marco del cuerpo (x, y, z).
        """
        self.params = params or GaitParams()
        self.names = tuple(leg_names)
        self.homes = [tuple(p) for p in homes]
        self.offset = [0.0 if n in GROUP_A else 0.5 for n in self.names]
        self.feet = [list(p) for p in self.homes]
        self.liftoff = [list(p) for p in self.homes]
        self.phase = 0.0
        self.walking = False
        self.v = (0.0, 0.0)   # velocidad que se está ejecutando (con rampa)
        self.wz = 0.0

    def _ramp(self, dt, v, wz):
        """Acerca la velocidad actual a la pedida sin pasar max_accel / max_alpha."""
        p = self.params
        dvx, dvy = v[0] - self.v[0], v[1] - self.v[1]
        dv = math.hypot(dvx, dvy)
        k = min(1.0, p.max_accel * dt / dv) if dv > 0 else 1.0
        self.v = (self.v[0] + k * dvx, self.v[1] + k * dvy)
        dw = wz - self.wz
        self.wz += max(-p.max_alpha * dt, min(p.max_alpha * dt, dw))
        return self.v, self.wz

    def _stance_move(self, foot, v, wz, t):
        """Mueve un pie apoyado como si el cuerpo avanzara durante t segundos."""
        x, y = foot[0], foot[1]
        foot[0] = x - (v[0] - wz * y) * t
        foot[1] = y - (v[1] + wz * x) * t

    def leg_phase(self, i):
        """Fase de la pata i en [0, 1)."""
        return (self.phase + self.offset[i]) % 1.0

    def in_stance(self, i):
        """Indica si la pata i está apoyada."""
        return self.leg_phase(i) < self.params.duty

    def at_home(self):
        """Indica si todos los pies están en su posición neutra."""
        tol = self.params.home_tol
        return all(math.dist(f, h) < tol for f, h in zip(self.feet, self.homes))

    def update(self, dt, v, wz):
        """
        Avanza la marcha dt segundos con velocidad v = (vx, vy) [m/s] y giro wz [rad/s].

        Devuelve la lista de posiciones de los pies (marco del cuerpo).
        """
        p = self.params
        v, wz, _ = limit_command(p, v, wz, self.homes)
        v, wz = self._ramp(dt, v, wz)
        moving = abs(v[0]) > 1e-6 or abs(v[1]) > 1e-6 or abs(wz) > 1e-6
        if not moving and not self.walking:
            return [tuple(f) for f in self.feet]
        if not moving and self.at_home():
            self.walking = False
            return [tuple(f) for f in self.feet]
        self.walking = True

        was_stance = [self.in_stance(i) for i in range(len(self.names))]
        self.phase = (self.phase + dt / p.period) % 1.0

        for i, home in enumerate(self.homes):
            foot = self.feet[i]
            d = step_vector(p, v, wz, home)
            target = (home[0] + d[0] / 2, home[1] + d[1] / 2, home[2])
            if self.in_stance(i):
                if not was_stance[i]:
                    # Aterriza en su destino y ya avanza lo que lleva de apoyo
                    # (la fase cruzó β entre dos pasos de tiempo).
                    self.feet[i] = foot = list(target)
                    self._stance_move(foot, v, wz, self.leg_phase(i) * p.period)
                    continue
                # El cuerpo se mueve v + ω ẑ × p; el pie apoyado, al revés.
                self._stance_move(foot, v, wz, dt)
            else:
                if was_stance[i]:
                    # Despega: se registra dónde habría terminado el apoyo justo
                    # en φ = β, no en el paso de tiempo anterior.
                    prev = (self.leg_phase(i) - dt / p.period) % 1.0
                    lo = list(foot)
                    self._stance_move(lo, v, wz, (p.duty - prev) * p.period)
                    self.liftoff[i] = lo
                u = (self.leg_phase(i) - p.duty) / (1.0 - p.duty)
                lo = self.liftoff[i]
                foot[0] = lo[0] + u * (target[0] - lo[0])
                foot[1] = lo[1] + u * (target[1] - lo[1])
                foot[2] = home[2] + p.step_height * math.sin(math.pi * u)
        return [tuple(f) for f in self.feet]
