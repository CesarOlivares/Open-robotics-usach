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
Dibuja el layout del hexápodo (planta y vista lateral) leyendo el URDF.

Todo sale del modelo: si cambia hexapod_params.xacro, la figura cambia.
Las patas se dibujan en una postura dada (por defecto, la de pie).

Uso (con el workspace compilado y sourceado):
    python3 analysis/plot_layout.py                     → docs/img/layout.png
    python3 analysis/plot_layout.py --pose zero -o x.png
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from hexapod_control.kinematics import ik, leg_geometry_from_urdf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.spatial import ConvexHull  # noqa: E402
import xacro  # noqa: E402

LEGS = ('lf', 'lm', 'lr', 'rf', 'rm', 'rr')
NAVY, ORANGE, DARK, GREY = '#002F6C', '#EA7600', '#26262B', '#9EA3AB'
COLORS = {'hexapod_navy': NAVY, 'hexapod_orange': ORANGE, 'hexapod_dark': DARK,
          'hexapod_grey': GREY, 'hexapod_green': '#1A8C40', 'hexapod_blue': '#2659D9'}


def rot_rpy(r, p, y):
    """Rotación roll-pitch-yaw del URDF."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    """Rotación de q en torno a un eje (Rodrigues)."""
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(q) * kx + (1 - math.cos(q)) * kx @ kx


def tf(elem):
    """Transformada 4×4 del <origin> de un elemento."""
    T = np.eye(4)
    o = elem.find('origin')
    if o is not None:
        T[:3, :3] = rot_rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in o.get('xyz', '0 0 0').split()]
    return T


def link_poses(root, q):
    """Pose de cada link en base_link para los ángulos q (dict junta → rad)."""
    poses = {'base_link': np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        rest = []
        for j in pending:
            parent = j.find('parent').get('link')
            if parent not in poses:
                rest.append(j)
                continue
            T = poses[parent] @ tf(j)
            if j.get('type') == 'revolute':
                R = np.eye(4)
                R[:3, :3] = rot_axis([float(v) for v in j.find('axis').get('xyz').split()],
                                     q.get(j.get('name'), 0.0))
                T = T @ R
            poses[j.find('child').get('link')] = T
        pending = rest
    return poses


def box_corners(size):
    """Las 8 esquinas de una caja centrada en el origen."""
    sx, sy, sz = (s / 2 for s in size)
    return np.array([[x, y, z, 1.0] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])


def draw(ax, root, poses, plane):
    """Proyecta cada visual (cajas y esferas) en el plano 'xy' o 'xz'."""
    i, j = (0, 1) if plane == 'xy' else (0, 2)
    items = []
    for link in root.findall('link'):
        for vis in link.findall('visual'):
            T = poses[link.get('name')] @ tf(vis)
            geom = vis.find('geometry')[0]
            mat = vis.find('material')
            color = COLORS.get(mat.get('name') if mat is not None else '', GREY)
            if geom.tag == 'box':
                pts = (T @ box_corners([float(v) for v in geom.get('size').split()]).T).T
                hull = ConvexHull(pts[:, [i, j]])
                poly = pts[hull.vertices][:, [i, j]]
                depth = pts[:, 2 if plane == 'xy' else 1].max()
                items.append((depth if plane == 'xy' else -pts[:, 1].min(), poly, color))
            elif geom.tag == 'sphere':
                c = T[:3, 3]
                items.append((c[2] if plane == 'xy' else -c[1],
                              plt.Circle((c[i] * 1000, c[j] * 1000),
                                         float(geom.get('radius')) * 1000), color))
    for _, shape, color in sorted(items, key=lambda t: t[0]):
        if isinstance(shape, plt.Circle):
            shape.set_facecolor(color)
            shape.set_edgecolor('white')
            shape.set_linewidth(0.4)
            ax.add_patch(shape)
        else:
            ax.fill(shape[:, 0] * 1000, shape[:, 1] * 1000, color=color,
                    ec='white' if color != NAVY else NAVY, lw=0.4)


def standing_pose(root, foot_out, foot_down):
    """Ángulos de la postura de pie: pie a foot_out del fémur y foot_down bajo él."""
    urdf_xml = ET.tostring(root, encoding='unicode')
    q = {}
    for leg in LEGS:
        g = leg_geometry_from_urdf(urdf_xml, leg)
        t1, t2, t3 = ik(g, (g.coxa + foot_out, 0.0, -foot_down))
        q.update({f'{leg}_coxa_joint': t1, f'{leg}_femur_joint': t2, f'{leg}_tibia_joint': t3})
    return q


def main():
    """Genera la figura de planta y vista lateral."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('--pose', choices=('stand', 'zero'), default='stand')
    ap.add_argument('-o', '--output', default=os.path.join(
        os.path.dirname(__file__), '..', 'docs', 'img', 'layout.png'))
    args = ap.parse_args()

    path = os.path.join(get_package_share_directory('hexapod_description'),
                        'urdf', 'hexapod.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path).toxml())
    q = standing_pose(root, 0.070, 0.090) if args.pose == 'stand' else {}
    poses = link_poses(root, q)

    fig, (top, side) = plt.subplots(1, 2, figsize=(12, 5.6),
                                    gridspec_kw={'width_ratios': [1, 1.15]})
    draw(top, root, poses, 'xy')
    draw(side, root, poses, 'xz')
    for leg in LEGS:
        x, y = poses[f'{leg}_foot_link'][:2, 3] * 1000
        top.annotate(leg.upper(), (x, y), xytext=(x * 1.2, y * 1.2), ha='center',
                     va='center', fontsize=9, color=NAVY, weight='bold')
    top.annotate('', xy=(150, 0), xytext=(105, 0),
                 arrowprops={'arrowstyle': '->', 'color': ORANGE, 'lw': 1.6})
    top.text(152, 6, 'frente (+x)', color=ORANGE, fontsize=9)
    ground = min(p[2, 3] for n, p in poses.items() if n.endswith('_foot_link')) * 1000 - 10
    side.axhline(ground, color=GREY, lw=1, ls='--')
    side.text(-230, ground + 3, 'suelo', color=GREY, fontsize=9)
    top.set_xlim(-290, 290)
    top.set_ylim(-290, 290)
    for ax, title, ylab in ((top, 'Planta (vista desde arriba)', 'y [mm]'),
                            (side, 'Vista lateral (desde la izquierda)', 'z [mm]')):
        ax.set_aspect('equal')
        ax.set_title(title, color=NAVY, fontsize=11)
        ax.set_xlabel('x [mm]')
        ax.set_ylabel(ylab)
        ax.grid(alpha=0.25)
    pose_txt = 'de pie (pie 70 mm afuera y 90 mm bajo el fémur)' if args.pose == 'stand' \
        else 'patas rectas'
    fig.suptitle(f'Layout del hexápodo desde el URDF — postura {pose_txt}',
                 color=NAVY, fontsize=12)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(f'figura: {os.path.abspath(args.output)}')


if __name__ == '__main__':
    main()
