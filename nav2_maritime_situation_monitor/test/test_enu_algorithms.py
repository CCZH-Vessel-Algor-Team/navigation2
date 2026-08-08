# Copyright 2026 vectorwang
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

from dataclasses import FrozenInstanceError
import math

from nav2_maritime_situation_monitor.enu_algorithms import (
    classify_encounter,
    classify_risk,
    compute_cpa,
    course_from_velocity,
    CPAResult,
    EncounterConfig,
    EncounterType,
    propagate_motion,
    relative_bearing,
    RiskLevel,
    RiskThreshold,
    RiskThresholds,
    validate_risk_thresholds,
    Vector2,
    VesselMotion,
)
import pytest


@pytest.fixture
def thresholds():
    return RiskThresholds(
        info=RiskThreshold(30.0, 120.0),
        warning=RiskThreshold(20.0, 30.0),
        critical=RiskThreshold(10.0, 10.0),
    )


def test_head_on_cpa():
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
    target = VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0))
    assert compute_cpa(own, target, 1e-6) == CPAResult(True, 0.0, 10.0)


def test_separating_target_has_negative_tcpa():
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(-1.0, 0.0))
    target = VesselMotion(Vector2(20.0, 0.0), Vector2(1.0, 0.0))
    result = compute_cpa(own, target, 1e-6)
    assert result.valid
    assert result.tcpa == -10.0


def test_equal_velocity_has_invalid_cpa_and_current_range():
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(2.0, 1.0))
    target = VesselMotion(Vector2(3.0, 4.0), Vector2(2.0, 1.0))
    assert compute_cpa(own, target, 1e-6) == CPAResult(False, 5.0, 0.0)


def test_relative_speed_at_positive_epsilon_has_valid_cpa():
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 0.0))
    target = VesselMotion(Vector2(3.0, 4.0), Vector2(0.05, 0.0))
    assert compute_cpa(own, target, 0.05) == CPAResult(True, 4.0, -60.0)


def test_zero_relative_speed_is_invalid_when_epsilon_is_zero():
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(2.0, 1.0))
    target = VesselMotion(Vector2(3.0, 4.0), Vector2(2.0, 1.0))
    assert compute_cpa(own, target, 0.0) == CPAResult(False, 5.0, 0.0)


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('field', ['position_x', 'position_y', 'velocity_x', 'velocity_y'])
def test_non_finite_motion_raises_value_error(value, field):
    values = {
        'position_x': 0.0,
        'position_y': 0.0,
        'velocity_x': 0.0,
        'velocity_y': 0.0,
    }
    values[field] = value
    own = VesselMotion(
        Vector2(values['position_x'], values['position_y']),
        Vector2(values['velocity_x'], values['velocity_y']),
    )
    target = VesselMotion(Vector2(1.0, 0.0), Vector2(0.0, 0.0))

    with pytest.raises(ValueError):
        compute_cpa(own, target, 1e-6)


@pytest.mark.parametrize('epsilon', [math.nan, math.inf, -math.inf, -0.01])
def test_invalid_relative_speed_epsilon_raises_value_error(epsilon):
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 0.0))
    target = VesselMotion(Vector2(1.0, 0.0), Vector2(0.0, 0.0))

    with pytest.raises(ValueError):
        compute_cpa(own, target, epsilon)


@pytest.mark.parametrize(
    ('cpa', 'expected'),
    [
        (CPAResult(True, 10.0, 10.0), RiskLevel.CRITICAL),
        (CPAResult(True, 20.0, 30.0), RiskLevel.WARNING),
        (CPAResult(True, 30.0, 120.0), RiskLevel.INFO),
    ],
)
def test_risk_threshold_boundaries_are_inclusive(cpa, expected, thresholds):
    assert classify_risk(cpa, thresholds) is expected


@pytest.mark.parametrize(
    ('cpa', 'expected'),
    [
        (CPAResult(True, 15.0, 10.0), RiskLevel.WARNING),
        (CPAResult(True, 5.0, 100.0), RiskLevel.INFO),
        (CPAResult(True, 1.0, -0.1), RiskLevel.SAFE),
        (CPAResult(False, 1.0, 0.0), RiskLevel.SAFE),
        (CPAResult(True, 30.1, 1.0), RiskLevel.SAFE),
        (CPAResult(True, 1.0, 120.1), RiskLevel.SAFE),
    ],
)
def test_risk_classification(cpa, expected, thresholds):
    assert classify_risk(cpa, thresholds) is expected


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf, -0.1])
@pytest.mark.parametrize(
    ('level', 'field'),
    [
        ('info', 'dcpa'),
        ('info', 'tcpa'),
        ('warning', 'dcpa'),
        ('warning', 'tcpa'),
        ('critical', 'dcpa'),
        ('critical', 'tcpa'),
    ],
)
def test_invalid_risk_threshold_values_raise_value_error(value, level, field, thresholds):
    values = {
        name: {'dcpa': threshold.dcpa, 'tcpa': threshold.tcpa}
        for name, threshold in (
            ('info', thresholds.info),
            ('warning', thresholds.warning),
            ('critical', thresholds.critical),
        )
    }
    values[level][field] = value
    invalid = RiskThresholds(
        info=RiskThreshold(**values['info']),
        warning=RiskThreshold(**values['warning']),
        critical=RiskThreshold(**values['critical']),
    )

    with pytest.raises(ValueError):
        validate_risk_thresholds(invalid)


@pytest.mark.parametrize(
    'thresholds',
    [
        RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(31.0, 30.0),
            critical=RiskThreshold(10.0, 10.0),
        ),
        RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(20.0, 121.0),
            critical=RiskThreshold(10.0, 10.0),
        ),
        RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(20.0, 30.0),
            critical=RiskThreshold(21.0, 10.0),
        ),
        RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(20.0, 30.0),
            critical=RiskThreshold(10.0, 31.0),
        ),
    ],
)
def test_non_nested_risk_thresholds_raise_value_error(thresholds):
    with pytest.raises(ValueError):
        validate_risk_thresholds(thresholds)


@pytest.fixture
def encounter_config():
    return EncounterConfig()


def test_encounter_config_defaults_and_frozen_behavior(encounter_config):
    assert encounter_config.head_on_bearing_rad == pytest.approx(math.radians(6.0))
    assert encounter_config.reciprocal_course_tolerance_rad == pytest.approx(
        math.radians(15.0)
    )
    assert encounter_config.stern_sector_rad == pytest.approx(math.radians(112.5))
    assert encounter_config.course_epsilon == 0.05

    with pytest.raises(FrozenInstanceError):
        encounter_config.course_epsilon = 0.1

    with pytest.raises(FrozenInstanceError):
        Vector2(1.0, 2.0).x = 3.0


@pytest.mark.parametrize(
    ('velocity', 'expected'),
    [
        (Vector2(1.0, 0.0), 0.0),
        (Vector2(0.0, 1.0), math.pi / 2.0),
        (Vector2(-1.0, 0.0), math.pi),
        (Vector2(0.0, -1.0), -math.pi / 2.0),
    ],
)
def test_course_from_velocity_uses_enu_cardinal_courses(velocity, expected):
    assert course_from_velocity(velocity, 0.05) == pytest.approx(expected)


def test_course_from_velocity_is_unknown_at_epsilon():
    assert course_from_velocity(Vector2(0.05, 0.0), 0.05) == 0.0


def test_zero_velocity_has_no_course_when_epsilon_is_zero():
    assert course_from_velocity(Vector2(0.0, 0.0), 0.0) is None


@pytest.mark.parametrize('epsilon', [math.nan, math.inf, -math.inf, -0.01])
def test_course_rejects_invalid_epsilon(epsilon):
    with pytest.raises(ValueError, match='epsilon'):
        course_from_velocity(Vector2(1.0, 0.0), epsilon)


def test_propagate_motion_uses_constant_global_velocity():
    motion = VesselMotion(Vector2(10.0, -3.0), Vector2(2.0, -0.5))
    assert propagate_motion(motion, 4.0) == VesselMotion(
        Vector2(18.0, -5.0),
        motion.velocity,
    )


@pytest.mark.parametrize('elapsed_seconds', [math.nan, math.inf, -math.inf, -0.01])
def test_propagate_motion_rejects_invalid_elapsed_time(elapsed_seconds):
    motion = VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
    with pytest.raises(ValueError, match='elapsed'):
        propagate_motion(motion, elapsed_seconds)


def test_propagate_motion_rejects_non_finite_motion():
    motion = VesselMotion(Vector2(math.nan, 0.0), Vector2(1.0, 0.0))
    with pytest.raises(ValueError, match='motion'):
        propagate_motion(motion, 1.0)


def test_propagate_motion_rejects_arithmetic_overflow():
    motion = VesselMotion(
        Vector2(1.0e308, -1.0e308),
        Vector2(1.0e308, -1.0e308),
    )
    with pytest.raises(ValueError, match='propagated position'):
        propagate_motion(motion, 2.0)


def test_relative_bearing_is_positive_to_port_and_wrapped():
    assert relative_bearing(0.0, Vector2(1.0, 1.0)) == pytest.approx(math.pi / 4.0)
    assert relative_bearing(0.0, Vector2(1.0, -1.0)) == pytest.approx(-math.pi / 4.0)
    assert relative_bearing(3.0 * math.pi / 4.0, Vector2(0.0, -1.0)) == pytest.approx(
        3.0 * math.pi / 4.0
    )


@pytest.mark.parametrize(
    ('own_velocity', 'target_position', 'target_velocity'),
    [
        (Vector2(1.0, 0.0), Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
        (Vector2(0.0, 1.0), Vector2(0.0, 20.0), Vector2(0.0, -1.0)),
        (Vector2(-1.0, 0.0), Vector2(-20.0, 0.0), Vector2(1.0, 0.0)),
        (Vector2(0.0, -1.0), Vector2(0.0, -20.0), Vector2(0.0, 1.0)),
    ],
)
def test_head_on_classification_for_all_cardinal_courses(
    own_velocity, target_position, target_velocity, encounter_config
):
    own = VesselMotion(Vector2(0.0, 0.0), own_velocity)
    target = VesselMotion(target_position, target_velocity)
    assert classify_encounter(own, target, encounter_config) is EncounterType.HEAD_ON


@pytest.mark.parametrize(
    ('target_position', 'target_velocity', 'expected'),
    [
        (Vector2(10.0, 10.0), Vector2(0.0, -1.0), EncounterType.CROSSING_LEFT),
        (Vector2(10.0, -10.0), Vector2(0.0, 1.0), EncounterType.CROSSING_RIGHT),
    ],
)
def test_crossing_classification_uses_port_positive_bearing(
    target_position, target_velocity, expected, encounter_config
):
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
    target = VesselMotion(target_position, target_velocity)
    assert classify_encounter(own, target, encounter_config) is expected


@pytest.mark.parametrize(
    ('own', 'target'),
    [
        (
            VesselMotion(Vector2(0.0, 0.0), Vector2(3.0, 0.0)),
            VesselMotion(Vector2(20.0, 0.0), Vector2(1.0, 0.0)),
        ),
        (
            VesselMotion(Vector2(20.0, 0.0), Vector2(1.0, 0.0)),
            VesselMotion(Vector2(0.0, 0.0), Vector2(3.0, 0.0)),
        ),
    ],
)
def test_both_overtaking_directions_are_merged(own, target, encounter_config):
    assert classify_encounter(own, target, encounter_config) is EncounterType.OVERTAKING


@pytest.mark.parametrize('slow_vessel', ['own', 'target'])
def test_vessel_below_course_epsilon_has_unknown_encounter(
    slow_vessel, encounter_config
):
    own_velocity = Vector2(0.049, 0.0) if slow_vessel == 'own' else Vector2(1.0, 0.0)
    target_velocity = (
        Vector2(-1.0, 0.0) if slow_vessel == 'own' else Vector2(0.049, 0.0)
    )
    own = VesselMotion(Vector2(0.0, 0.0), own_velocity)
    target = VesselMotion(Vector2(20.0, 0.0), target_velocity)
    assert classify_encounter(own, target, encounter_config) is EncounterType.UNKNOWN


@pytest.mark.parametrize('bearing_deg', [6.0, -6.0, 112.5, -112.5])
def test_exact_sector_boundaries_are_unknown(bearing_deg, encounter_config):
    bearing = math.radians(bearing_deg)
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
    target = VesselMotion(
        Vector2(100.0 * math.cos(bearing), 100.0 * math.sin(bearing)),
        Vector2(-1.0, 0.0) if abs(bearing_deg) == 6.0 else Vector2(1.0, 0.0),
    )
    assert classify_encounter(own, target, encounter_config) is EncounterType.UNKNOWN


def test_overtaking_has_precedence_over_head_on():
    config = EncounterConfig(
        head_on_bearing_rad=math.pi,
        reciprocal_course_tolerance_rad=math.pi,
    )
    own = VesselMotion(Vector2(0.0, 0.0), Vector2(3.0, 0.0))
    target = VesselMotion(Vector2(20.0, 0.0), Vector2(1.0, 0.0))
    assert classify_encounter(own, target, config) is EncounterType.OVERTAKING


def test_encounter_classification_is_independent_of_cpa_risk(encounter_config):
    own = VesselMotion(Vector2(20.0, 0.0), Vector2(3.0, 0.0))
    target = VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))
    assert compute_cpa(own, target, 1e-6).tcpa < 0.0
    assert classify_encounter(own, target, encounter_config) is EncounterType.OVERTAKING


@pytest.mark.parametrize(
    ('own', 'target', 'expected_cpa', 'expected_encounter'),
    [
        (
            VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 2.0)),
            VesselMotion(Vector2(0.0, 20.0), Vector2(0.0, -2.0)),
            CPAResult(True, 0.0, 5.0),
            EncounterType.HEAD_ON,
        ),
        (
            VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 2.0)),
            VesselMotion(Vector2(20.0, 20.0), Vector2(-2.0, 0.0)),
            CPAResult(True, 0.0, 10.0),
            EncounterType.CROSSING_RIGHT,
        ),
        (
            VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 2.0)),
            VesselMotion(Vector2(-20.0, 20.0), Vector2(2.0, 0.0)),
            CPAResult(True, 0.0, 10.0),
            EncounterType.CROSSING_LEFT,
        ),
        (
            VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 3.0)),
            VesselMotion(Vector2(0.0, 20.0), Vector2(0.0, 1.0)),
            CPAResult(True, 0.0, 10.0),
            EncounterType.OVERTAKING,
        ),
        (
            VesselMotion(Vector2(0.0, 20.0), Vector2(0.0, 1.0)),
            VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 3.0)),
            CPAResult(True, 0.0, 10.0),
            EncounterType.OVERTAKING,
        ),
    ],
)
def test_embedded_reference_fixtures_preserve_cpa_and_semantics(
    own, target, expected_cpa, expected_encounter, encounter_config
):
    assert compute_cpa(own, target, 1e-6) == expected_cpa
    assert classify_encounter(own, target, encounter_config) is expected_encounter
