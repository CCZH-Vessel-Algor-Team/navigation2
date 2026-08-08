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
    EncounterConfig,
    EncounterType,
    RiskLevel,
    RiskThreshold,
    RiskThresholds,
    Vector2,
    VesselMotion,
)
from nav2_maritime_situation_monitor.report_builder import (
    AssessmentConfig,
    build_reports,
    SituationResult,
    TrackedMotion,
)
import pytest


@pytest.fixture
def config():
    return AssessmentConfig(
        risk_thresholds=RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(20.0, 30.0),
            critical=RiskThreshold(10.0, 10.0),
        ),
        relative_speed_epsilon=1e-6,
        encounter_config=EncounterConfig(),
    )


@pytest.fixture
def ownship():
    return VesselMotion(Vector2(0.0, 0.0), Vector2(1.0, 0.0))


def test_build_reports_preserves_all_results_in_uuid_byte_order(ownship, config):
    lower_id = bytes.fromhex('00000000000000000000000000000001')
    higher_id = bytes.fromhex('000000000000000000000000000000ff')
    equal_velocity = VesselMotion(Vector2(3.0, 4.0), Vector2(1.0, 0.0))
    head_on = VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0))

    reports = build_reports(
        ownship,
        (
            TrackedMotion(higher_id, head_on),
            TrackedMotion(lower_id, equal_velocity),
        ),
        config,
    )

    assert reports == (
        SituationResult(
            target_id=lower_id,
            cpa_valid=False,
            dcpa=5.0,
            tcpa=0.0,
            risk=RiskLevel.SAFE,
            encounter=EncounterType.OVERTAKING,
        ),
        SituationResult(
            target_id=higher_id,
            cpa_valid=True,
            dcpa=0.0,
            tcpa=10.0,
            risk=RiskLevel.CRITICAL,
            encounter=EncounterType.HEAD_ON,
        ),
    )


def test_report_order_is_independent_of_target_input_order(ownship, config):
    first = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000001'),
        VesselMotion(Vector2(10.0, 10.0), Vector2(0.0, -1.0)),
    )
    second = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000002'),
        VesselMotion(Vector2(10.0, -10.0), Vector2(0.0, 1.0)),
    )

    assert build_reports(ownship, (second, first), config) == build_reports(
        ownship, (first, second), config
    )


def test_non_finite_target_is_skipped_without_dropping_valid_targets(
    ownship, config
):
    valid_id = bytes.fromhex('00000000000000000000000000000001')
    invalid_id = bytes.fromhex('00000000000000000000000000000002')
    valid = TrackedMotion(
        valid_id,
        VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
    )
    invalid = TrackedMotion(
        invalid_id,
        VesselMotion(Vector2(math.nan, 0.0), Vector2(-1.0, 0.0)),
    )

    reports = build_reports(ownship, (invalid, valid), config)

    assert len(reports) == 1
    assert reports[0].target_id == valid_id


def test_non_finite_shared_ownship_raises_value_error(config):
    ownship = VesselMotion(Vector2(math.nan, 0.0), Vector2(1.0, 0.0))
    target = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000001'),
        VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
    )

    with pytest.raises(ValueError, match='motion must be finite'):
        build_reports(ownship, (target,), config)


def test_invalid_shared_risk_thresholds_raise_value_error(ownship, config):
    invalid = AssessmentConfig(
        risk_thresholds=RiskThresholds(
            info=RiskThreshold(30.0, 120.0),
            warning=RiskThreshold(31.0, 30.0),
            critical=RiskThreshold(10.0, 10.0),
        ),
        relative_speed_epsilon=config.relative_speed_epsilon,
        encounter_config=config.encounter_config,
    )
    target = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000001'),
        VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
    )

    with pytest.raises(ValueError, match='risk thresholds'):
        build_reports(ownship, (target,), invalid)


def test_invalid_shared_encounter_config_raises_value_error(ownship, config):
    invalid = AssessmentConfig(
        risk_thresholds=config.risk_thresholds,
        relative_speed_epsilon=config.relative_speed_epsilon,
        encounter_config=EncounterConfig(course_epsilon=math.nan),
    )
    target = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000001'),
        VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
    )

    with pytest.raises(ValueError, match='encounter config'):
        build_reports(ownship, (target,), invalid)


@pytest.mark.parametrize('epsilon', [math.nan, math.inf, -1e-6])
def test_invalid_shared_relative_speed_epsilon_raises_value_error(
    epsilon, ownship, config
):
    invalid = AssessmentConfig(
        risk_thresholds=config.risk_thresholds,
        relative_speed_epsilon=epsilon,
        encounter_config=config.encounter_config,
    )
    target = TrackedMotion(
        bytes.fromhex('00000000000000000000000000000001'),
        VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0)),
    )

    with pytest.raises(ValueError, match='relative speed epsilon'):
        build_reports(ownship, (target,), invalid)


def test_duplicate_target_ids_raise_value_error(ownship, config):
    target_id = bytes.fromhex('00000000000000000000000000000001')
    motion = VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0))

    with pytest.raises(ValueError, match='duplicate target UUID'):
        build_reports(
            ownship,
            (TrackedMotion(target_id, motion), TrackedMotion(target_id, motion)),
            config,
        )


def test_duplicate_id_is_rejected_when_first_target_is_invalid(ownship, config):
    target_id = bytes.fromhex('00000000000000000000000000000001')
    invalid = VesselMotion(Vector2(math.nan, 0.0), Vector2(-1.0, 0.0))
    valid = VesselMotion(Vector2(20.0, 0.0), Vector2(-1.0, 0.0))

    with pytest.raises(ValueError, match='duplicate target UUID'):
        build_reports(
            ownship,
            (TrackedMotion(target_id, invalid), TrackedMotion(target_id, valid)),
            config,
        )


@pytest.mark.parametrize('target_id', [b'', b'short', b'x' * 15, b'x' * 17])
def test_tracked_motion_requires_exactly_16_uuid_bytes(target_id):
    motion = VesselMotion(Vector2(0.0, 0.0), Vector2(0.0, 0.0))

    with pytest.raises(ValueError, match='16 bytes'):
        TrackedMotion(target_id, motion)


def test_report_domain_types_are_frozen(ownship, config):
    target_id = bytes.fromhex('00000000000000000000000000000001')
    tracked = TrackedMotion(target_id, ownship)
    result = SituationResult(
        target_id,
        False,
        0.0,
        0.0,
        RiskLevel.SAFE,
        EncounterType.UNKNOWN,
    )

    with pytest.raises(FrozenInstanceError):
        tracked.target_id = b'x' * 16
    with pytest.raises(FrozenInstanceError):
        result.dcpa = 1.0
    with pytest.raises(FrozenInstanceError):
        config.relative_speed_epsilon = 0.1
