#!/usr/bin/env python3
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
Estudio de dimensiones del hexápodo (E8): de los componentes reales a las cotas.

Pasos (el modelo está en hexapod_control/design_study.py):
    1. Empaque: L1 y R mínimos con las medidas reales de servos, batería y PCA.
    2. Altura: H mínima para la altura libre pedida.
    3. Torque vs. estabilidad: barrido de d (distancia fémur → pie).
    4. Alcance: barrido de L2 × L3 para cada d; se elige el centro de la
       región que cumple.
    5. Sensibilidad: 4,8 V, piezas más pesadas.
Genera 3 figuras en docs/img/ y analysis/resultado_estudio.json.

Uso (con el workspace sourceado):
    python3 analysis/dimension_study.py
"""

from dataclasses import asdict, replace
import json
import math
import os

from hexapod_control.design_study import (choose, Components, Design, evaluate,
                                          foot_load, joint_torques, min_body_radius,
                                          min_coxa_length, min_femur_length,
                                          Requirements, robustness, stride_points,
                                          sweep_lengths, torque_safety)
from hexapod_control.kinematics import ik
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, '..', 'docs', 'img')
NAVY, ORANGE, GREY, GREEN, RED = '#002F6C', '#EA7600', '#9EA3AB', '#2E7D32', '#C62828'

L2S = np.round(np.arange(0.045, 0.1201, 0.005), 3)
L3S = np.round(np.arange(0.080, 0.2001, 0.005), 3)
DS = np.round(np.arange(0.040, 0.0751, 0.005), 3)
CURRENT = Design(0.045, 0.080, 0.120, 0.100, 0.070, 0.090)   # diseño de partida (E0–E7)


def round_up(x, step=0.005):
    """Redondea hacia arriba a múltiplos de `step` (5 mm)."""
    return math.ceil(round(x / step, 6)) * step


def main():
    """Corre el estudio, imprime el resumen y guarda figuras y resultados."""
    comp, req = Components(), Requirements()
    tau_d = req.design_torque(comp)
    print(f'Carga por pie (trípode): F = M g / 3 = {foot_load(comp):.2f} N; '
          f'torque de diseño τd = {comp.servo_stall} / {req.safety_factor} = {tau_d:.2f} N·m')

    # 1. Empaque
    l1_min, l2_min, r_min = min_coxa_length(comp), min_femur_length(comp), min_body_radius(comp)
    L1 = round_up(l1_min + 0.004)        # +4 mm para el rodamiento del adaptador coaxial
    R = round_up(r_min)
    print(f'1. Empaque: L1 ≥ {l1_min * 1000:.1f} mm → L1 = {L1 * 1000:.0f} mm; '
          f'L2 ≥ {l2_min * 1000:.1f} mm; R ≥ {r_min * 1000:.1f} mm → R = {R * 1000:.0f} mm')

    # 2. Altura
    H = round_up(req.clearance + comp.battery_below_base - comp.foot_radius)
    print(f'2. Altura: H ≥ {(req.clearance + comp.battery_below_base - comp.foot_radius) * 1000:.1f}'
          f' mm → H = {H * 1000:.0f} mm (altura libre '
          f'{(H + comp.foot_radius - comp.battery_below_base) * 1000:.1f} mm)')

    # 3 y 4. Barrido de d y de L2 × L3
    print('3–4. Barrido de d, y de L2 × L3 para cada d:')
    per_d = {}
    for d in DS:
        sw = sweep_lengths(Design(L1, 0, 0, R, d, H), L2S, L3S, comp, req)
        best = choose(sw, comp, req)
        per_d[d] = (sw, best)
        if best:
            dd, r = best
            print(f'   d = {d * 1000:.0f} mm: {len(sw.designs):3d} cumplen; centro L2 = '
                  f'{dd.L2 * 1000:.0f}, L3 = {dd.L3 * 1000:.0f} mm; FS torque '
                  f'{torque_safety(r, comp):.2f}; holgura {robustness(r, req):.2f}; '
                  f'estabilidad {r["estabilidad [m]"][0] * 1000:.0f} mm')
        else:
            print(f'   d = {d * 1000:.0f} mm: ninguno cumple')

    # Elección de d: la mayor con FS ≥ 2,1 (margen para la masa de las piezas
    # que aún no existen) y holgura ≥ 1 (tolera 10 mm de error).
    ok = [d for d, (_, b) in per_d.items()
          if b and torque_safety(b[1], comp) >= 2.1 and robustness(b[1], req) >= 1.0]
    d_star = max(ok)
    chosen, res = per_d[d_star][1]
    print(f'\nELEGIDO: L1 = {chosen.L1 * 1000:.0f}, L2 = {chosen.L2 * 1000:.0f}, '
          f'L3 = {chosen.L3 * 1000:.0f}, R = {chosen.R * 1000:.0f} mm; '
          f'postura d = {chosen.d * 1000:.0f}, H = {chosen.H * 1000:.0f} mm')
    for k, v in res.items():
        if k != 'feasible':
            print(f'   {k:28s} {v[0]:8.3f}  (mín {v[1]:.3f})  {"✅" if v[2] else "❌"}')
    print(f'   FS de torque {torque_safety(res, comp):.2f}; holgura {robustness(res, req):.2f}')

    print('\nDiseño de partida (E0–E7):')
    r0 = evaluate(CURRENT, comp, req)
    for k, v in r0.items():
        if k != 'feasible' and not v[2]:
            print(f'   NO cumple {k}: {v[0]:.3f} (mín {v[1]:.3f})')

    # 5. Sensibilidad
    print('\n5. Sensibilidad del diseño elegido:')
    sens = {}
    for name, c in (('servos a 4,8 V (0,92 N·m)', replace(comp, servo_stall=0.92)),
                    ('piezas de la pata +50 %', replace(comp, femur_bracket_mass=0.015,
                                                        tibia_mass=0.0225)),
                    ('robot +200 g (más estructura)', replace(comp, total_mass=2.076))):
        r = evaluate(chosen, c, req)
        fs = torque_safety(r, c)
        sens[name] = fs
        print(f'   {name:32s} FS = {fs:.2f}')

    os.makedirs(IMG, exist_ok=True)
    fig_torque(per_d, d_star, comp, req, chosen)
    fig_map(per_d[d_star][0], chosen, req)
    fig_leg(chosen, req)

    out = {'elegido': asdict(chosen),
           'restricciones': {k: list(v) for k, v in res.items() if k != 'feasible'},
           'fs_torque': torque_safety(res, comp), 'holgura': robustness(res, req),
           'sensibilidad_fs': sens,
           'requisitos': asdict(req), 'componentes': asdict(comp)}
    with open(os.path.join(HERE, 'resultado_estudio.json'), 'w') as f:
        json.dump(out, f, indent=2, ensure_ascii=False,
                  default=lambda o: o.item() if hasattr(o, 'item') else str(o))
    print('\nFiguras en docs/img/estudio_*.png; resultados en analysis/resultado_estudio.json')


def fig_torque(per_d, d_star, comp, req, chosen):
    """Figura 1: torque del fémur y estabilidad en función de d."""
    ds = np.linspace(0.030, 0.090, 61)
    base = replace(chosen)
    tf = [max(joint_torques(replace(base, d=d), rho, -base.H, comp)[0]
              for rho in (d - req.stride / 2, d + req.stride / 2)) for d in ds]
    stab = [(base.R + base.L1 + d) / 2 - req.stride / 2 for d in ds]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    ax.plot(ds * 1000, tf, color=NAVY, lw=2.2, label='torque del fémur (peor punto del paso)')
    ax.axhline(comp.servo_stall, color=RED, ls='--', lw=1.2, label='bloqueo MG996R a 6 V (1,08)')
    ax.axhline(comp.servo_stall / req.safety_factor, color=ORANGE, ls='--', lw=1.5,
               label='torque de diseño, FS = 2 (0,54)')
    ax.axhline(0.92 / req.safety_factor, color=ORANGE, ls=':', lw=1.2,
               label='torque de diseño a 4,8 V (0,46)')
    ax.axvline(d_star * 1000, color=GREEN, lw=1.5)
    ax.text(d_star * 1000 + 0.6, 0.08, f'elegido\nd = {d_star * 1000:.0f} mm', color=GREEN)
    ax.axvline(70, color=GREY, lw=1.2, ls='-.')
    ax.text(70.6, 0.08, 'partida\nd = 70 mm', color=GREY)
    ax.set_xlabel('d: distancia horizontal eje del fémur → pie [mm]')
    ax.set_ylabel('torque [N·m]')
    ax.set_ylim(0, 1.2)
    ax2 = ax.twinx()
    ax2.plot(ds * 1000, np.array(stab) * 1000, color=GREEN, lw=1.5, alpha=0.7,
             label='margen de estabilidad')
    ax2.axhline(req.stability_min * 1000, color=GREEN, ls=':', lw=1)
    ax2.set_ylabel('margen de estabilidad [mm]', color=GREEN)
    ax2.set_ylim(0, 120)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=8)
    ax.grid(alpha=0.25)
    ax.set_title('Abrir la pata da estabilidad pero cuesta torque: el fémur fija d máx.',
                 color=NAVY)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'estudio_torque_vs_d.png'), dpi=150)
    plt.close(fig)


def fig_map(sweep, chosen, req):
    """Figura 2: mapa L2 × L3 de qué cumple y con cuánta holgura."""
    Z = np.full((len(L3S), len(L2S)), np.nan)
    reason = {}
    for (L2, L3), r in sweep.grid.items():
        i, j = list(L3S).index(L3), list(L2S).index(L2)
        if r['feasible']:
            Z[i, j] = robustness(r, req)
        else:
            reason[(i, j)] = next(k for k, v in r.items() if k != 'feasible' and not v[2])
    fig, ax = plt.subplots(figsize=(8.5, 6))
    ext = [L2S[0] * 1000 - 2.5, L2S[-1] * 1000 + 2.5, L3S[0] * 1000 - 2.5, L3S[-1] * 1000 + 2.5]
    im = ax.imshow(Z, origin='lower', extent=ext, aspect='auto', cmap='Greens', vmin=0, vmax=2)
    fail_colors = {'alcance [m]': '#F3D9C4', 'rodilla sobre el fémur [m]': '#D7DCE6',
                   'margen a los topes [rad]': '#E8C8C8', 'L2 ≥ 2 servos [m]': '#EEEEEE',
                   'torque fémur [N·m]': '#F5B7B1', 'torque tibia [N·m]': '#F5B7B1'}
    seen = set()
    for (i, j), k in reason.items():
        ax.add_patch(plt.Rectangle((L2S[j] * 1000 - 2.5, L3S[i] * 1000 - 2.5), 5, 5,
                                   color=fail_colors.get(k, '#EEEEEE'),
                                   label=None if k in seen else f'no cumple: {k.split(" [")[0]}'))
        seen.add(k)
    ax.plot(chosen.L2 * 1000, chosen.L3 * 1000, '*', ms=18, color=ORANGE, mec='k',
            label=f'elegido ({chosen.L2 * 1000:.0f}, {chosen.L3 * 1000:.0f})')
    ax.plot(80, 120, 'X', ms=11, color=GREY, mec='k', label='partida (80, 120)')
    plt.colorbar(im, ax=ax, label='holgura mínima (× 10 mm o 0,1 rad)')
    ax.set_xlabel('L2 fémur [mm]')
    ax.set_ylabel('L3 tibia [mm]')
    ax.set_title(f'Qué largos cumplen todo (L1 = {chosen.L1 * 1000:.0f}, R = '
                 f'{chosen.R * 1000:.0f}, d = {chosen.d * 1000:.0f}, H = {chosen.H * 1000:.0f} mm)',
                 color=NAVY, fontsize=11)
    ax.legend(loc='upper right', fontsize=7.5)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'estudio_mapa_largos.png'), dpi=150)
    plt.close(fig)


def fig_leg(design, req):
    """Figura 3: la pata elegida de costado, con su espacio de trabajo y el paso."""
    geom = design.geometry(req)
    fig, ax = plt.subplots(figsize=(9, 5.2))
    # Espacio de trabajo (plano de la pata, coxa en 0) dentro de los límites
    pts = []
    for t2 in np.linspace(*req.femur_limits, 90):
        for t3 in np.linspace(*req.tibia_limits, 90):
            x = design.L1 + design.L2 * math.cos(t2) + design.L3 * math.cos(t2 + t3)
            z = design.L2 * math.sin(t2) + design.L3 * math.sin(t2 + t3)
            pts.append((x, z))
    pts = np.array(pts) * 1000
    ax.scatter(pts[:, 0], pts[:, 1], s=1, color=GREY, alpha=0.25,
               label='alcance dentro de los límites')
    ground = -(design.H + 0.010) * 1000
    ax.axhline(ground, color='k', lw=1)
    ax.text(5, ground - 8, 'suelo', fontsize=9)
    ax.add_patch(plt.Rectangle((-30, -24.5), 30 + 15, 49, color=NAVY,
                               alpha=0.25, label='cuerpo (placas)'))
    bat = 47.5
    ax.add_patch(plt.Rectangle((-30, -bat), 30, 22, color='#2659D9', alpha=0.35,
                               label='batería'))
    # Postura en los 3 puntos del apoyo y el arco de vuelo
    home = design.L1 + design.d
    for dx, col, lab in ((-req.stride / 2, '#7B4A2A', 'apoyo: pie atrás/adentro'),
                         (0.0, ORANGE, 'postura neutra'),
                         (req.stride / 2, NAVY, 'apoyo: pie adelante/afuera')):
        a = ik(geom, (home + dx, 0.0, -design.H))
        k = (design.L1 + design.L2 * math.cos(a[1]), design.L2 * math.sin(a[1]))
        f = (k[0] + design.L3 * math.cos(a[1] + a[2]), k[1] + design.L3 * math.sin(a[1] + a[2]))
        xs = np.array([0, design.L1, k[0], f[0]]) * 1000
        zs = np.array([0, 0, k[1], f[1]]) * 1000
        ax.plot(xs, zs, '-o', color=col, lw=2.5, ms=5, label=lab)
    u = np.linspace(0, 1, 40)
    ax.plot((home + (u - 0.5) * req.stride) * 1000,
            (-design.H + req.swing_height * np.sin(math.pi * u)) * 1000,
            '--', color=GREEN, lw=1.5, label=f'arco de vuelo (h = {req.swing_height * 1000:.0f} mm)')
    ax.set_aspect('equal')
    ax.set_xlim(-35, 260)
    ax.set_ylim(-130, 140)
    ax.set_xlabel('distancia desde el eje de la coxa [mm]')
    ax.set_ylabel('altura [mm]')
    ax.set_title(f'Pata elegida: L1 = {design.L1 * 1000:.0f}, L2 = {design.L2 * 1000:.0f}, '
                 f'L3 = {design.L3 * 1000:.0f} mm (vista lateral)', color=NAVY)
    ax.legend(loc='upper right', fontsize=7.5)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'estudio_pata.png'), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
