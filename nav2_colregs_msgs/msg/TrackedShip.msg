# TrackedShip.msg
# Represents a tracked ship (Target Ship) — one entry in a TrackedShipList.
# Frame is inherited from TrackedShipList.header.frame_id.

unique_identifier_msgs/UUID target_id   # deterministic ID (UUID5 from model name)
geometry_msgs/Pose  pose                # position + orientation (in list frame_id)
geometry_msgs/Twist twist               # velocity (linear + angular)
float64             radius              # collision radius (meters)
uint32              ais_mmsi            # matched AIS MMSI; 0 when unknown or unavailable
