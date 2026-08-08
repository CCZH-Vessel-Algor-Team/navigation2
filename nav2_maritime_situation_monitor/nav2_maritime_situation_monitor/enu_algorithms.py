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

"""ROS-independent maritime calculations in REP-103 ENU coordinates."""

from dataclasses import dataclass
from enum import IntEnum
import math


@dataclass(frozen=True)
class Vector2:
    """A planar vector whose x and y axes point east and north, respectively."""

    x: float
    y: float

    def __add__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector2') -> 'Vector2':
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector2':
        return Vector2(self.x * scalar, self.y * scalar)

    def dot(self, other: 'Vector2') -> float:
        return self.x * other.x + self.y * other.y

    def norm(self) -> float:
        return math.hypot(self.x, self.y)


@dataclass(frozen=True)
class VesselMotion:
    position: Vector2
    velocity: Vector2


@dataclass(frozen=True)
class CPAResult:
    valid: bool
    dcpa: float
    tcpa: float


@dataclass(frozen=True)
class RiskThreshold:
    dcpa: float
    tcpa: float


@dataclass(frozen=True)
class RiskThresholds:
    info: RiskThreshold
    warning: RiskThreshold
    critical: RiskThreshold


class RiskLevel(IntEnum):
    SAFE = 0
    INFO = 1
    WARNING = 2
    CRITICAL = 3


class EncounterType(IntEnum):
    UNKNOWN = 0
    HEAD_ON = 1
    OVERTAKING = 2
    CROSSING_LEFT = 3
    CROSSING_RIGHT = 4


@dataclass(frozen=True)
class EncounterConfig:
    head_on_bearing_rad: float = math.radians(6.0)
    reciprocal_course_tolerance_rad: float = math.radians(15.0)
    stern_sector_rad: float = math.radians(112.5)
    course_epsilon: float = 0.05


def propagate_motion(motion: VesselMotion, elapsed_seconds: float) -> VesselMotion:
    """Propagate planar motion at constant global velocity."""
    values = (
        motion.position.x,
        motion.position.y,
        motion.velocity.x,
        motion.velocity.y,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('motion must be finite')
    if not math.isfinite(elapsed_seconds) or elapsed_seconds < 0.0:
        raise ValueError('elapsed time must be finite and non-negative')
    position = motion.position + motion.velocity * elapsed_seconds
    if not all(math.isfinite(value) for value in (position.x, position.y)):
        raise ValueError('propagated position must be finite')
    return VesselMotion(position, motion.velocity)


def compute_cpa(
    ownship: VesselMotion,
    target: VesselMotion,
    relative_speed_epsilon: float,
) -> CPAResult:
    values = (
        ownship.position.x,
        ownship.position.y,
        ownship.velocity.x,
        ownship.velocity.y,
        target.position.x,
        target.position.y,
        target.velocity.x,
        target.velocity.y,
        relative_speed_epsilon,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('CPA inputs must be finite')
    if relative_speed_epsilon < 0.0:
        raise ValueError('relative speed epsilon must be non-negative')

    relative_position = target.position - ownship.position
    relative_velocity = target.velocity - ownship.velocity
    speed_squared = relative_velocity.x ** 2 + relative_velocity.y ** 2
    if speed_squared == 0.0 or speed_squared < relative_speed_epsilon ** 2:
        return CPAResult(False, relative_position.norm(), 0.0)

    tcpa = -relative_position.dot(relative_velocity) / speed_squared
    dcpa = (relative_position + relative_velocity * tcpa).norm()
    return CPAResult(True, dcpa, tcpa)


def validate_risk_thresholds(thresholds: RiskThresholds) -> None:
    values = (
        thresholds.info.dcpa,
        thresholds.info.tcpa,
        thresholds.warning.dcpa,
        thresholds.warning.tcpa,
        thresholds.critical.dcpa,
        thresholds.critical.tcpa,
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError('risk thresholds must be finite and non-negative')

    if not (
        thresholds.critical.dcpa <= thresholds.warning.dcpa <= thresholds.info.dcpa
        and thresholds.critical.tcpa <= thresholds.warning.tcpa <= thresholds.info.tcpa
    ):
        raise ValueError('risk thresholds must be nested from critical to info')


def classify_risk(cpa: CPAResult, thresholds: RiskThresholds) -> RiskLevel:
    validate_risk_thresholds(thresholds)
    if not cpa.valid or cpa.tcpa < 0.0:
        return RiskLevel.SAFE

    for level, threshold in (
        (RiskLevel.CRITICAL, thresholds.critical),
        (RiskLevel.WARNING, thresholds.warning),
        (RiskLevel.INFO, thresholds.info),
    ):
        if cpa.dcpa <= threshold.dcpa and cpa.tcpa <= threshold.tcpa:
            return level

    return RiskLevel.SAFE


def course_from_velocity(velocity: Vector2, epsilon: float = 0.05) -> float | None:
    """Return the ENU course angle, or None when speed cannot define a course."""
    if not math.isfinite(epsilon) or epsilon < 0.0:
        raise ValueError('course epsilon must be finite and non-negative')
    speed = velocity.norm()
    if speed == 0.0 or speed < epsilon:
        return None
    return math.atan2(velocity.y, velocity.x)


def relative_bearing(observer_course: float, relative_position: Vector2) -> float:
    """Return a wrapped bearing with positive angles to the observer's left."""
    bearing = math.atan2(relative_position.y, relative_position.x) - observer_course
    return (bearing + math.pi) % (2.0 * math.pi) - math.pi


def classify_encounter(
    ownship: VesselMotion,
    target: VesselMotion,
    config: EncounterConfig = EncounterConfig(),
) -> EncounterType:
    """Classify moving-vessel geometry independently of CPA risk."""
    own_course = course_from_velocity(ownship.velocity, config.course_epsilon)
    target_course = course_from_velocity(target.velocity, config.course_epsilon)
    if own_course is None or target_course is None:
        return EncounterType.UNKNOWN

    own_to_target = target.position - ownship.position
    if own_to_target.norm() == 0.0:
        return EncounterType.UNKNOWN

    own_bearing = relative_bearing(own_course, own_to_target)
    target_bearing = relative_bearing(
        target_course,
        ownship.position - target.position,
    )

    if (
        math.isclose(
            abs(own_bearing), config.head_on_bearing_rad, rel_tol=0.0, abs_tol=1e-12
        )
        or math.isclose(
            abs(own_bearing), config.stern_sector_rad, rel_tol=0.0, abs_tol=1e-12
        )
        or math.isclose(
            abs(target_bearing), config.stern_sector_rad, rel_tol=0.0, abs_tol=1e-12
        )
    ):
        return EncounterType.UNKNOWN

    if (
        abs(own_bearing) > config.stern_sector_rad
        or abs(target_bearing) > config.stern_sector_rad
    ):
        return EncounterType.OVERTAKING

    course_difference = abs(relative_bearing(own_course, target.velocity))
    reciprocal_error = abs(math.pi - course_difference)
    if (
        abs(own_bearing) < config.head_on_bearing_rad
        and reciprocal_error <= config.reciprocal_course_tolerance_rad
    ):
        return EncounterType.HEAD_ON

    if config.head_on_bearing_rad < own_bearing < config.stern_sector_rad:
        return EncounterType.CROSSING_LEFT
    if -config.stern_sector_rad < own_bearing < -config.head_on_bearing_rad:
        return EncounterType.CROSSING_RIGHT
    return EncounterType.UNKNOWN
