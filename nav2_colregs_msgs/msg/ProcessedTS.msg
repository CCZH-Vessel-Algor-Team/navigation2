std_msgs/Header   header
unique_identifier_msgs/UUID target_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float64           radius

# ── CPA: snapshot at CURRENT OS velocity ──
float64           tcpa
float64           dcpa
# Manager alert: current inflated-domain intrusion OR a qualifying future CPA
# at the current OS velocity. Used for primary-target candidacy, not as a
# geometric clearance certificate for a new requested heading.
bool              has_threat

# ── Diagnostic sampled LVO Collision Cone ──
# OS heading intervals [0, 2π) in header.frame_id, using manager radius inflation,
# current OS speed and constant TS velocity. Narrow cones can fall between samples.
#   []          = no sampled heading was unsafe (NOT an all-headings-safe proof)
#   [[a, b]]    = these heading intervals are unsafe
#   [[0, 2π]]   = ALL headings collide (inescapable at current speed)
# Avoidance recomputes continuous collision checks with its own inflation factor;
# it must not simply take inclusive complements of these sampled intervals.
float64[] collision_cone_min
float64[] collision_cone_max

# ── Reserved for future COLREGS encounter classification ──
# Always UNKNOWN(0). Will be determined at BT layer, not here.
# Do not depend on this field.
int8 UNKNOWN=0
int8 HEAD_ON=1
int8 CROSSING_STARBOARD=2
int8 CROSSING_PORT=3
int8 OVERTAKING_GIVEWAY=4
int8 OVERTAKING_STANDON=5
int8 encounter_type 0
