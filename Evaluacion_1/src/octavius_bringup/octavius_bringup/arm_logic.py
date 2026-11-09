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
Cálculos del control articular de Octavius, sin dependencias de ROS.

Está separado del nodo arm_commander para poder probar cada fórmula con
tests unitarios (test/test_arm_logic.py) sin levantar ROS ni Gazebo.

Unidades: ángulos en rad, velocidades en rad/s, tiempos en s.
"""

from typing import Dict, Tuple
import xml.etree.ElementTree as ET

# Límites de una articulación: (mínimo, máximo).
Limits = Tuple[float, float]


def parse_joint_limits(urdf_xml: str) -> Dict[str, Limits]:
    """
    Extrae los límites de las articulaciones móviles desde el texto del URDF.

    Lee cada <joint> de tipo revolute o prismatic y devuelve sus atributos
    <limit lower=".." upper="..">. Las juntas fixed y continuous no tienen
    recorrido limitado y se omiten.

    Así los límites se definen en un solo lugar (octavius_params.xacro) y el
    nodo los recibe a través de /robot_description, sin copiarlos a mano.
    """
    limits: Dict[str, Limits] = {}
    root = ET.fromstring(urdf_xml)
    for joint in root.iter('joint'):
        if joint.get('type') not in ('revolute', 'prismatic'):
            continue
        limit = joint.find('limit')
        if limit is None:
            continue
        limits[joint.get('name')] = (float(limit.get('lower', 0.0)),
                                     float(limit.get('upper', 0.0)))
    return limits


def clamp(value: float, limits: Limits) -> float:
    """Satura value dentro del intervalo [mínimo, máximo]."""
    lower, upper = limits
    return max(lower, min(upper, value))


def apply_deadzone(axis: float, deadzone: float) -> float:
    """
    Anula las lecturas pequeñas de un eje del joystick.

    Un joystick analógico en reposo casi nunca entrega exactamente 0; sin
    zona muerta el brazo se movería solo, lentamente.
    """
    return 0.0 if abs(axis) < deadzone else axis


def integrate_velocity(target: float, axis: float, max_velocity: float,
                       dt: float, limits: Limits) -> float:
    """
    Avanza un ángulo objetivo según la inclinación del joystick.

        q_objetivo ← clamp(q_objetivo + v · ω_max · Δt, q_min, q_max)

    con v ∈ [-1, 1] la inclinación del eje y ω_max la velocidad máxima.
    Ejemplo: v = 1, ω_max = 0.5 rad/s, Δt = 0.02 s → avanza 0.01 rad.
    """
    return clamp(target + axis * max_velocity * dt, limits)


def apply_step(target: float, direction: float, step: float,
               limits: Limits) -> float:
    """
    Mueve un ángulo objetivo un paso fijo (una pulsación de tecla).

        q_objetivo ← clamp(q_objetivo + d · paso, q_min, q_max)

    con d = +1 o -1 según la tecla.
    """
    return clamp(target + direction * step, limits)


def step_toward(current: float, goal: float, max_delta: float) -> float:
    """
    Acerca current a goal, avanzando como máximo max_delta.

    Se usa para volver a la pose home de forma suave, a velocidad acotada,
    en vez de saltar de golpe.
    """
    delta = goal - current
    if abs(delta) <= max_delta:
        return goal
    return current + max_delta if delta > 0 else current - max_delta
