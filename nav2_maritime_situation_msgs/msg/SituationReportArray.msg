# stamp is the common evaluation time to which all vessel states were propagated.
# frame_id is the global ENU assessment frame used for every target calculation.
std_msgs/Header header
# False on missing/stale input, failed transforms or incomplete assessment.
# An empty array is obstacle-free only when this is true.
bool observations_valid
SituationReport[] targets
