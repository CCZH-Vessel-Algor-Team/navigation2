# Risk enum values. risk_level must be one of RISK_SAFE through RISK_CRITICAL.
uint8 RISK_SAFE=0
uint8 RISK_INFO=1
uint8 RISK_WARNING=2
uint8 RISK_CRITICAL=3

# Encounter enum values. encounter_type must be one of ENCOUNTER_UNKNOWN through
# ENCOUNTER_CROSSING_RIGHT.
uint8 ENCOUNTER_UNKNOWN=0
uint8 ENCOUNTER_HEAD_ON=1
uint8 ENCOUNTER_OVERTAKING=2
uint8 ENCOUNTER_CROSSING_LEFT=3
uint8 ENCOUNTER_CROSSING_RIGHT=4

unique_identifier_msgs/UUID target_id
# Whether DCPA/TCPA were computed from nonzero relative velocity at or above
# the configured tolerance.
# If cpa_valid=false: tcpa=0, dcpa=current range, risk_level=RISK_SAFE.
bool cpa_valid
# Distance at closest point of approach, in meters.
float64 dcpa
# Time from the array header's evaluation time to CPA, in seconds.
# Negative values indicate that CPA is in the past.
float64 tcpa
# Must contain one of the ENCOUNTER_* constants defined above.
uint8 encounter_type
# Must contain one of the RISK_* constants defined above.
uint8 risk_level
