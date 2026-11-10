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

"""Tests unitarios de los cálculos de arm_logic (sin ROS)."""

from octavius_bringup.arm_logic import (apply_deadzone, apply_step, clamp,
                                        integrate_velocity, parse_joint_limits,
                                        soft_limits, step_toward)
import pytest

URDF = """
<robot name="test">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="fixed_j" type="fixed"><parent link="a"/><child link="b"/></joint>
  <joint name="rev_j" type="revolute">
    <parent link="b"/><child link="c"/>
    <limit lower="-1.5" upper="2.0" velocity="1" effort="1"/>
  </joint>
  <joint name="pri_j" type="prismatic">
    <parent link="c"/><child link="d"/>
    <limit lower="0.0" upper="0.025" velocity="1" effort="1"/>
  </joint>
</robot>
"""


def test_parse_joint_limits_reads_only_movable_joints():
    limits = parse_joint_limits(URDF)
    assert limits == {'rev_j': (-1.5, 2.0), 'pri_j': (0.0, 0.025)}


def test_soft_limits_shrink_both_sides():
    assert soft_limits((-1.0, 2.0), 0.02) == pytest.approx((-0.98, 1.98))


def test_soft_limits_reject_margin_larger_than_range():
    with pytest.raises(ValueError):
        soft_limits((0.0, 0.03), 0.02)


def test_clamp():
    assert clamp(3.0, (-1.0, 1.0)) == 1.0
    assert clamp(-3.0, (-1.0, 1.0)) == -1.0
    assert clamp(0.3, (-1.0, 1.0)) == 0.3


def test_deadzone_cancels_small_readings():
    assert apply_deadzone(0.05, 0.1) == 0.0
    assert apply_deadzone(-0.09, 0.1) == 0.0
    assert apply_deadzone(0.5, 0.1) == 0.5


def test_integrate_velocity_example_from_documentation():
    # v = 1, ω_max = 0.5 rad/s, Δt = 0.02 s → +0.01 rad
    assert integrate_velocity(0.0, 1.0, 0.5, 0.02, (-3.0, 3.0)) == \
        pytest.approx(0.01)
    assert integrate_velocity(0.0, -1.0, 0.5, 0.02, (-3.0, 3.0)) == \
        pytest.approx(-0.01)


def test_integrate_velocity_never_exceeds_limits():
    target = 0.0
    for _ in range(1000):
        target = integrate_velocity(target, 1.0, 0.5, 0.02, (-1.0, 1.0))
    assert target == 1.0


def test_apply_step():
    assert apply_step(0.0, 1.0, 0.05, (-1.0, 1.0)) == pytest.approx(0.05)
    assert apply_step(0.98, 1.0, 0.05, (-1.0, 1.0)) == 1.0


def test_step_toward_reaches_goal_without_overshoot():
    assert step_toward(1.0, 0.0, 0.3) == pytest.approx(0.7)
    assert step_toward(-1.0, 0.0, 0.3) == pytest.approx(-0.7)
    assert step_toward(0.1, 0.0, 0.3) == 0.0


def test_keyboard_mapping():
    from octavius_bringup.keyboard_teleop import key_to_joy
    assert key_to_joy('w') == ([1.0, 0.0, 0.0], [0, 0])
    assert key_to_joy('F') == ([0.0, -1.0, 0.0], [0, 0])
    assert key_to_joy('q') == ([0.0, 0.0, -1.0], [0, 0])
    assert key_to_joy('a') == ([0.0, 0.0, 0.0], [1, 0])
    assert key_to_joy('h') == ([0.0, 0.0, 0.0], [0, 1])
    assert key_to_joy('x') is None
