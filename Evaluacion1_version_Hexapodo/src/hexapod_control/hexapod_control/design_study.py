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
Estudio de dimensiones del hexápodo (E8), sin ROS.

Pregunta: con 18 MG996R, la LiPo 3S y la electrónica dada, ¿qué largos
L1, L2, L3 y qué radio R permiten caminar con margen de torque, sin choques,
con altura libre y de forma estable?

Variables:
    L1, L2, L3  largos coxa, fémur, tibia (de eje a eje; L3 hasta la punta del pie)
    R           centro del cuerpo → eje de cada coxa
    d           postura: distancia horizontal del eje del FÉMUR al pie
    H           postura: altura del eje de la coxa sobre el centro del pie

Requisitos de uso (Requirements): paso S, altura de vuelo h, altura libre
mínima, factor de seguridad del torque, margen de estabilidad mínimo.

Modelo de carga (validado en Gazebo en E4: 0,375 N·m medido = calculado):
    en la marcha trípode 3 patas sostienen todo el peso: F = M g / 3 por pie.
    Torque del fémur = F · (distancia horizontal fémur → pie)
                       − peso propio de la pata (servo de rodilla, piezas,
                         tibia), que tira hacia el otro lado.
    Torque de la tibia = F · (distancia horizontal rodilla → pie) − peso de la tibia.
El peor punto del paso es el pie en el extremo exterior: d + S/2.
"""

from dataclasses import dataclass, field, replace
import math

from hexapod_control.kinematics import ik, LegGeometry, UnreachableError

G = 9.81


@dataclass(frozen=True)
class Components:
    """Medidas y masas reales [m, kg] (las mismas de hexapod_params.xacro)."""

    servo_mass: float = 0.055          # medido, con adaptadores
    servo_x: float = 0.0407            # largo
    servo_y: float = 0.0197            # ancho
    servo_z: float = 0.0429            # alto (incluye el eje)
    servo_shaft_offset: float = 0.010  # eje a ~10 mm de un extremo
    servo_stall: float = 1.08          # N·m a 6 V
    battery: tuple = (0.137, 0.043, 0.022)
    pca: tuple = (0.062, 0.025, 0.015)
    plate_margin: float = 0.015        # la placa sobresale del eje de la coxa
    battery_below_base: float = 0.0475  # cara inferior de la batería bajo base_link
    foot_radius: float = 0.010
    total_mass: float = 1.876          # masa del modelo (E2)
    femur_bracket_mass: float = 0.010  # hipótesis
    tibia_mass: float = 0.015          # hipótesis
    foot_mass: float = 0.001


@dataclass(frozen=True)
class Requirements:
    """Lo que se le pide al robot."""

    stride: float = 0.060          # S: paso completo (v máx 0,10 m/s con β T = 0,6 s)
    swing_height: float = 0.030    # h: altura de vuelo del pie
    clearance: float = 0.060       # altura libre mínima bajo el cuerpo (batería)
    safety_factor: float = 2.0     # torque de diseño = bloqueo / FS
    stability_min: float = 0.040   # margen de estabilidad estático mínimo
    feet_gap_min: float = 0.030    # distancia mínima entre pies vecinos
    servo_gap_min: float = 0.005   # holgura mínima entre servos de fémur vecinos
    reach_max: float = 0.90        # D ≤ 0,9 (L2 + L3): no estirar la pata del todo
    limit_margin: float = 0.10     # rad: no llegar al tope de ninguna articulación
    coxa_limit: float = math.pi / 4
    femur_limits: tuple = (-math.pi / 2, math.pi / 2)
    tibia_limits: tuple = (-2.5, 0.6)

    def design_torque(self, comp):
        """Torque de diseño τd = torque de bloqueo / FS."""
        return comp.servo_stall / self.safety_factor


@dataclass(frozen=True)
class Design:
    """Una combinación de dimensiones y postura."""

    L1: float
    L2: float
    L3: float
    R: float
    d: float
    H: float

    def geometry(self, req=Requirements()):
        """Geometría de la pata con los límites articulares."""
        return LegGeometry(self.L1, self.L2, self.L3,
                           (-req.coxa_limit, req.coxa_limit),
                           req.femur_limits, req.tibia_limits)


# ---------------------------------------------------------------- empaque

def min_coxa_length(comp=Components(), clearance=0.005):
    """
    L1 mínimo: el servo del fémur (centrado en su eje) no debe tocar la placa.

    La placa sobresale `plate_margin` del eje de la coxa; el servo del fémur
    ocupa ± servo_x/2 alrededor de su eje, que está a L1.
    """
    return comp.plate_margin + comp.servo_x / 2 + clearance


def min_femur_length(comp=Components(), clearance=0.005):
    """
    Calcula L2 mínimo: los dos servos a lo largo del fémur no deben tocarse.

    El servo del fémur (en su eje) y el de la rodilla (en la rodilla) van
    uno tras otro a lo largo del fémur.
    """
    return comp.servo_x + clearance


def hexagon_contains(apothem, x, y):
    """Indica si (x, y) cae dentro del hexágono (lado plano al frente) de apotema dada."""
    for k in range(6):
        a = math.radians(60 * k)
        if x * math.cos(a) + y * math.sin(a) > apothem + 1e-12:
            return False
    return True


def packaging(R, comp=Components(), clearance=0.005):
    """
    Holguras de empaque del cuerpo para un radio R [m] (positivo = cabe).

    battery      la batería (137 × 43) cabe bajo la placa inferior
    electronics  las 2 PCA9685 (lado a lado en el centro) no tocan los
                 servos de las coxas, que entran desde los vértices
    """
    apothem = (R + comp.plate_margin) * math.sqrt(3) / 2
    bx, by = comp.battery[0] / 2, comp.battery[1] / 2
    battery = apothem - bx - clearance if hexagon_contains(apothem, bx, by) else \
        -(bx - apothem)
    pca_corner = math.hypot(comp.pca[0] / 2, comp.pca[1] + clearance)
    # extremo interior del servo de coxa: R − (servo_x − shaft_offset)
    servo_inner = R - (comp.servo_x - comp.servo_shaft_offset)
    electronics = servo_inner - pca_corner - clearance
    return {'battery': battery, 'electronics': electronics}


def min_body_radius(comp=Components(), step=0.001):
    """R mínimo que hace caber batería y electrónica."""
    R = 0.04
    while min(packaging(R, comp).values()) < 0:
        R += step
    return R


# ---------------------------------------------------------------- cargas

def foot_load(comp=Components()):
    """F por pie de apoyo en la marcha trípode: M g / 3."""
    return comp.total_mass * G / 3


def _leg_points(design, angles):
    """Rodilla y pie en el plano de la pata, medidos desde el eje del fémur (x, z)."""
    t2, t3 = angles[1], angles[2]
    knee = (design.L2 * math.cos(t2), design.L2 * math.sin(t2))
    foot = (knee[0] + design.L3 * math.cos(t2 + t3), knee[1] + design.L3 * math.sin(t2 + t3))
    return knee, foot


def joint_torques(design, rho, z, comp=Components()):
    """
    Torques estáticos [N·m] de fémur y tibia con el pie apoyado en (rho, z).

    rho: distancia horizontal eje del fémur → pie; z: altura del pie
    respecto del eje (negativa). Usa la IK real (rodilla arriba).
    """
    geom = LegGeometry(design.L1, design.L2, design.L3)
    angles = ik(geom, (design.L1 + rho, 0.0, z), check_limits=False)
    knee, foot = _leg_points(design, angles)
    F = foot_load(comp)
    m_femur_side = comp.femur_bracket_mass * knee[0] / 2 + comp.servo_mass * knee[0]
    tibia_cx = (knee[0] + foot[0]) / 2
    self_femur = G * (m_femur_side + comp.tibia_mass * tibia_cx + comp.foot_mass * foot[0])
    self_tibia = G * (comp.tibia_mass * (tibia_cx - knee[0])
                      + comp.foot_mass * (foot[0] - knee[0]))
    femur = F * foot[0] - self_femur
    tibia = F * (foot[0] - knee[0]) - self_tibia
    return abs(femur), abs(tibia)


def max_stance_distance(comp=Components(), req=Requirements(), self_relief=0.0):
    """
    Calcula d máximo por torque del fémur (análisis rápido, sin peso propio).

    F (d + S/2) ≤ τd  →  d ≤ τd / F − S/2. El peso propio de la pata alivia
    algo; `self_relief` [N·m] permite sumarlo.
    """
    return (req.design_torque(comp) + self_relief) / foot_load(comp) - req.stride / 2


# ---------------------------------------------------------------- evaluación

def stride_points(design, req, n=16):
    """
    Puntos del pie (marco de la pata) que recorre el paso, en cualquier dirección.

    Apoyo: un círculo de radio S/2 alrededor de la posición neutra.
    Vuelo: el mismo círculo a la altura h y el punto neutro a la altura h.
    """
    home = design.L1 + design.d
    pts = []
    for k in range(n):
        a = 2 * math.pi * k / n
        dx, dy = req.stride / 2 * math.cos(a), req.stride / 2 * math.sin(a)
        pts.append(('apoyo', (home + dx, dy, -design.H)))
        pts.append(('vuelo', (home + dx, dy, -design.H + req.swing_height * 0.5)))
    pts.append(('vuelo', (home, 0.0, -design.H + req.swing_height)))
    return pts


def evaluate(design, comp=Components(), req=Requirements()):
    """
    Calcula todas las restricciones de un diseño.

    Devuelve dict nombre → (valor, mínimo exigido, cumple), y 'feasible'.
    Un margen se expresa en la unidad natural de cada criterio.
    """
    res = {}
    tau_d = req.design_torque(comp)
    geom = design.geometry(req)
    lim = geom.limits()

    # Alcance y límites, en todos los puntos del paso
    worst_limit = math.inf
    worst_reach = math.inf
    knee_low = math.inf
    reach_ok = True
    for phase, p in stride_points(design, req):
        rho = math.hypot(p[0], p[1]) - design.L1
        D = math.hypot(rho, p[2])
        worst_reach = min(worst_reach, req.reach_max * (design.L2 + design.L3) - D)
        try:
            ang = ik(geom, p, check_limits=False)
        except UnreachableError:
            reach_ok = False
            continue
        for a, (lo, hi) in zip(ang, lim):
            worst_limit = min(worst_limit, a - lo, hi - a)
        if phase == 'apoyo':
            knee_low = min(knee_low, design.L2 * math.sin(ang[1]))
    res['alcance [m]'] = (worst_reach if reach_ok else -1.0, 0.0,
                          reach_ok and worst_reach >= 0)
    res['margen a los topes [rad]'] = (worst_limit, req.limit_margin,
                                       reach_ok and worst_limit >= req.limit_margin)
    # Postura de araña: la rodilla sobre el eje del fémur en todo el apoyo, así
    # su servo no roza el suelo y la tibia trabaja casi vertical.
    res['rodilla sobre el fémur [m]'] = (knee_low if reach_ok else -1.0, 0.0,
                                         reach_ok and knee_low >= 0.0)

    # Torques en el peor punto del apoyo (extremos interior y exterior)
    tf = tt = 0.0
    if reach_ok:
        for rho in (design.d - req.stride / 2, design.d + req.stride / 2):
            f, t = joint_torques(design, rho, -design.H, comp)
            tf, tt = max(tf, f), max(tt, t)
    res['torque fémur [N·m]'] = (tf, tau_d, reach_ok and tf <= tau_d)
    res['torque tibia [N·m]'] = (tt, tau_d, reach_ok and tt <= tau_d)

    # Altura libre: cara inferior de la batería sobre el suelo
    clear = design.H + comp.foot_radius - comp.battery_below_base
    res['altura libre [m]'] = (clear, req.clearance, clear >= req.clearance - 1e-9)

    # Estabilidad estática: distancia del centro de masa al borde del trípode
    # (triángulo equilátero de radio r_f, desplazado S/2 en la peor dirección)
    rf = design.R + design.L1 + design.d
    stab = rf / 2 - req.stride / 2
    res['estabilidad [m]'] = (stab, req.stability_min, stab >= req.stability_min)

    # Pies vecinos (a 60°): en el peor caso se acercan S
    gap = rf - req.stride
    res['pies vecinos [m]'] = (gap, req.feet_gap_min, gap >= req.feet_gap_min)

    # Servos de fémur vecinos: centros a R + L1, cada coxa gira hacia el otro
    alpha = math.atan2(req.stride / 2, design.L1 + design.d)
    chord = 2 * (design.R + design.L1) * math.sin(math.radians(30) - alpha)
    sgap = chord - comp.servo_z
    res['servos vecinos [m]'] = (sgap, req.servo_gap_min, sgap >= req.servo_gap_min)

    # Empaque del cuerpo
    pk = packaging(design.R, comp)
    res['batería cabe [m]'] = (pk['battery'], 0.0, pk['battery'] >= 0)
    res['electrónica cabe [m]'] = (pk['electronics'], 0.0, pk['electronics'] >= 0)
    l1_min = min_coxa_length(comp)
    res['L1 ≥ servo [m]'] = (design.L1 - l1_min, 0.0, design.L1 >= l1_min - 1e-9)
    l2_min = min_femur_length(comp)
    res['L2 ≥ 2 servos [m]'] = (design.L2 - l2_min, 0.0, design.L2 >= l2_min - 1e-9)

    res['feasible'] = all(v[2] for k, v in res.items())
    return res


def torque_safety(res, comp=Components()):
    """Factor de seguridad real del peor servo (bloqueo / torque)."""
    worst = max(res['torque fémur [N·m]'][0], res['torque tibia [N·m]'][0])
    return comp.servo_stall / worst if worst > 0 else math.inf


@dataclass
class SweepResult:
    """Resultado del barrido de L2 y L3."""

    designs: list = field(default_factory=list)   # (Design, res) factibles
    grid: dict = field(default_factory=dict)      # (L2, L3) → res


def sweep_lengths(base, L2s, L3s, comp=Components(), req=Requirements()):
    """Evalúa todas las combinaciones (L2, L3) con el resto del diseño fijo."""
    out = SweepResult()
    for L2 in L2s:
        for L3 in L3s:
            d = replace(base, L2=L2, L3=L3)
            r = evaluate(d, comp, req)
            out.grid[(L2, L3)] = r
            if r['feasible']:
                out.designs.append((d, r))
    return out


# Tolerancias con que se normalizan las holguras geométricas en choose():
# lo que podría desviarse al fabricar y montar las piezas.
TOL_LENGTH = 0.010   # m
TOL_ANGLE = 0.10     # rad


def robustness(res, req=Requirements()):
    """
    Holgura geométrica mínima, en unidades de tolerancia de fabricación.

    1 significa que el diseño soporta un error de 10 mm (o 0,1 rad) en su
    criterio más ajustado antes de dejar de cumplir.
    """
    return min(res['alcance [m]'][0] / TOL_LENGTH,
               res['rodilla sobre el fémur [m]'][0] / TOL_LENGTH,
               (res['margen a los topes [rad]'][0] - req.limit_margin) / TOL_ANGLE)


def choose(sweep, comp=Components(), req=Requirements()):
    """
    El diseño en el centro de la región que cumple.

    Entre los factibles, el de mayor holgura mínima (robustness): un diseño
    en el borde de la región deja de cumplir con el primer error de
    fabricación. Desempate: la pata más corta (menos masa e inercia).
    """
    if not sweep.designs:
        return None
    return max(sweep.designs,
               key=lambda dr: (round(robustness(dr[1], req), 2),
                               -round(dr[0].L2 + dr[0].L3, 4)))
