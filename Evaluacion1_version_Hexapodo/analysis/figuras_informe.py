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
Figuras del informe que salen del código del proyecto (sin simulación).

    docs/img/marcha_tripode.png   fases de las 6 patas y trayectoria de un pie,
                                  calculadas con hexapod_control/gait.py
    docs/img/velocidad_d11.png    velocidad pedida vs. medida (D11), con los
                                  valores medidos por scripts/medir_marcha.py (E8)

Uso: python3 analysis/figuras_informe.py   (con el workspace sourceado)
"""

import os

from ament_index_python.packages import get_package_share_directory
from hexapod_control.gait import GaitParams, GROUP_A, TripodGait
from hexapod_control.robot_model import HexapodModel, LEG_NAMES
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import xacro  # noqa: E402

IMG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'img')
ORANGE, GREY, GREEN = '#E58503', '#9EA3AB', '#2E7D32'
NAVY = '#2464B0'

# D11 medido con scripts/medir_marcha.py tras el estudio de dimensiones (E8)
D11 = [('Adelante', 0.080, 0.073, 'm/s'), ('Atrás', 0.080, 0.073, 'm/s'),
       ('Lateral', 0.060, 0.053, 'm/s'), ('Giro izq.', 0.400, 0.342, 'rad/s'),
       ('Giro der.', 0.400, 0.342, 'rad/s')]


def fig_gait():
    """Fases de apoyo/vuelo de las 6 patas y la trayectoria de un pie."""
    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    model = HexapodModel(xacro.process_file(path).toxml())
    l1 = model.legs[0].geom.coxa
    params = GaitParams(period=1.0, duty=0.6, max_accel=100.0, max_alpha=100.0)
    gait = TripodGait(LEG_NAMES, model.neutral_feet(l1 + 0.060, 0.100), params)
    dt, v = 0.01, (0.08, 0.0)
    for _ in range(200):                       # régimen permanente
        gait.update(dt, v, 0.0)
    t, stance, feet = [], [], []
    for k in range(300):
        f = gait.update(dt, v, 0.0)
        t.append(k * dt)
        stance.append([gait.in_stance(i) for i in range(6)])
        feet.append(f)
    t, stance, feet = np.array(t), np.array(stance), np.array(feet)

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6.2), height_ratios=[1, 1.15])
    order = ['lf', 'rm', 'lr', 'rf', 'lm', 'rr']
    for row, name in enumerate(order):
        i = LEG_NAMES.index(name)
        col = NAVY if name in GROUP_A else ORANGE
        on = stance[:, i]
        start = None
        for k in range(len(t) + 1):
            if k < len(t) and on[k] and start is None:
                start = k
            if (k == len(t) or not on[k]) and start is not None:
                a1.barh(row, t[k - 1] - t[start] + dt, left=t[start], color=col, height=0.6)
                start = None
    a1.set_yticks(range(6), [n.upper() for n in order])
    a1.invert_yaxis()
    a1.set_xlim(0, 3)
    a1.set_title('Marcha trípode (β = 0,6, T = 1 s): barra = pata apoyada. Azul grupo A '
                 '(LF, RM, LR), naranjo grupo B (RF, LM, RR).\nFranjas verdes: las 6 patas '
                 'apoyadas (10 % del ciclo en cada cambio de grupo)', fontsize=10)
    both = stance[:, [LEG_NAMES.index(n) for n in order]].sum(axis=1) == 6
    for k in np.where(np.diff(both.astype(int)) == 1)[0]:
        a1.axvspan(t[k], t[k] + 0.1, color=GREEN, alpha=0.15)
    i = LEG_NAMES.index('lf')
    home = model.neutral_feet(l1 + 0.060, 0.100)[i]
    a2.plot(t, (feet[:, i, 0] - home[0]) * 1000, color=NAVY, lw=2,
            label='x del pie LF respecto de su posición neutra')
    a2.plot(t, (feet[:, i, 2] - home[2]) * 1000, color=ORANGE, lw=2, label='altura del pie LF')
    a2.set_xlim(0, 3)
    a2.set_xlabel('tiempo [s]')
    a2.set_ylabel('[mm]')
    a2.legend(fontsize=8, loc='upper right')
    a2.grid(alpha=0.25)
    a2.set_title('Pie LF a 0,08 m/s: en el apoyo retrocede (empuja); en el vuelo sube '
                 '30 mm y vuelve adelante', fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'marcha_tripode.png'), dpi=150)
    plt.close(fig)


def fig_d11():
    """Velocidad pedida vs. medida en Gazebo."""
    fig, ax = plt.subplots(figsize=(8, 3.6))
    x = np.arange(len(D11))
    ped = [d[1] / (0.08 if d[3] == 'm/s' else 0.4) * 100 for d in D11]
    med = [d[2] / d[1] * 100 for d in D11]
    ax.bar(x - 0.2, [100] * len(D11), 0.4, color=GREY, label='pedido (100 %)')
    ax.bar(x + 0.2, med, 0.4, color=NAVY, label='medido en Gazebo')
    for k, d in enumerate(D11):
        ax.text(k + 0.2, med[k] + 2, f'{d[2]:.3f}\n{d[3]}', ha='center', fontsize=8)
        ax.text(k - 0.2, 102, f'{d[1]:.2f}', ha='center', fontsize=8, color='#555555')
    ax.axhline(80, color=ORANGE, ls='--', lw=1.2, label='mínimo aceptado (error < 20 %)')
    ax.set_xticks(x, [d[0] for d in D11])
    ax.set_ylabel('% de lo pedido')
    ax.set_ylim(0, 140)
    ax.legend(fontsize=8, loc='upper center', ncol=3, frameon=False)
    ax.set_title('D11: el robot camina a 85–91 % de la velocidad pedida', fontsize=10)
    ax.grid(axis='y', alpha=0.25)
    del ped
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'velocidad_d11.png'), dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    fig_gait()
    fig_d11()
    print('figuras en docs/img/: marcha_tripode.png, velocidad_d11.png')
