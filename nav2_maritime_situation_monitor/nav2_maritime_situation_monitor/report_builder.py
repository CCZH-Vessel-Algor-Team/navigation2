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

"""Pure construction of deterministic all-target maritime situation reports."""

from dataclasses import dataclass
import math
from typing import Iterable

from .enu_algorithms import (
    classify_encounter,
    classify_risk,
    compute_cpa,
    EncounterConfig,
    EncounterType,
    RiskLevel,
    RiskThresholds,
    validate_risk_thresholds,
    VesselMotion,
)


@dataclass(frozen=True)
class TrackedMotion:
    target_id: bytes
    motion: VesselMotion

    def __post_init__(self) -> None:
        if not isinstance(self.target_id, bytes) or len(self.target_id) != 16:
            raise ValueError('target UUID must be exactly 16 bytes')


@dataclass(frozen=True)
class SituationResult:
    target_id: bytes
    cpa_valid: bool
    dcpa: float
    tcpa: float
    risk: RiskLevel
    encounter: EncounterType


@dataclass(frozen=True)
class AssessmentConfig:
    risk_thresholds: RiskThresholds
    relative_speed_epsilon: float
    encounter_config: EncounterConfig = EncounterConfig()


def _validate_shared_inputs(
    ownship: VesselMotion,
    config: AssessmentConfig,
) -> None:
    motion_values = (
        ownship.position.x,
        ownship.position.y,
        ownship.velocity.x,
        ownship.velocity.y,
    )
    if not all(math.isfinite(value) for value in motion_values):
        raise ValueError('motion must be finite')

    validate_risk_thresholds(config.risk_thresholds)

    if (
        not math.isfinite(config.relative_speed_epsilon)
        or config.relative_speed_epsilon < 0.0
    ):
        raise ValueError('relative speed epsilon must be finite and non-negative')

    encounter = config.encounter_config
    encounter_values = (
        encounter.head_on_bearing_rad,
        encounter.reciprocal_course_tolerance_rad,
        encounter.stern_sector_rad,
        encounter.course_epsilon,
    )
    if not all(math.isfinite(value) for value in encounter_values):
        raise ValueError('encounter config values must be finite')
    if not (
        0.0 <= encounter.head_on_bearing_rad <= math.pi
        and 0.0 <= encounter.reciprocal_course_tolerance_rad <= math.pi
        and 0.0 <= encounter.stern_sector_rad <= math.pi
        and encounter.course_epsilon >= 0.0
    ):
        raise ValueError('encounter config values are out of range')


def build_reports(
    ownship: VesselMotion,
    targets: Iterable[TrackedMotion],
    config: AssessmentConfig,
) -> tuple[SituationResult, ...]:
    _validate_shared_inputs(ownship, config)

    reports = []
    seen_ids = set()

    for target in targets:
        if target.target_id in seen_ids:
            raise ValueError('duplicate target UUID')
        seen_ids.add(target.target_id)

        try:
            cpa = compute_cpa(
                ownship,
                target.motion,
                config.relative_speed_epsilon,
            )
            risk = classify_risk(cpa, config.risk_thresholds)
            encounter = classify_encounter(
                ownship,
                target.motion,
                config.encounter_config,
            )
        except ValueError:
            continue

        reports.append(
            SituationResult(
                target_id=target.target_id,
                cpa_valid=cpa.valid,
                dcpa=cpa.dcpa,
                tcpa=cpa.tcpa,
                risk=risk,
                encounter=encounter,
            )
        )

    return tuple(sorted(reports, key=lambda report: report.target_id))
