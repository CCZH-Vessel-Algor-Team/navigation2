std_msgs/Header   header
unique_identifier_msgs/UUID target_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float64           radius

# ── CPA: snapshot at CURRENT OS velocity ──
float64           tcpa
float64           dcpa
# Naive CPA check: TCPA < horizon AND DCPA < safe_dist at CURRENT velocity.
# True = "I am on a collision course right now."
# Use as alert trigger only. Do NOT use for path geometry decisions.
bool              has_threat

# ── LVO Collision Cone: pure kinematic ──
# OS heading intervals [0, 2π) in map frame that would lead to collision,
# assuming OS maintains its current speed and TS velocity is constant.
#   []          = no heading leads to collision (safe at all headings)
#   [[a, b]]    = these heading intervals are unsafe
#   [[0, 2π]]   = ALL headings collide (inescapable at current speed)
# Consumer takes complement of union to find safe headings.
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
