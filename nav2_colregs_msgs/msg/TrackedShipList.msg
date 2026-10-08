# TrackedShipList.msg
# Batch of tracked ships published by a fusion node.
# Complete snapshot: absent targets are removed, including an explicitly empty list.
# Poses AND linear velocities use header.frame_id. header.stamp is the observation
# time, not a timestamp refreshed while replaying an old measurement.

std_msgs/Header header
TrackedShip[] ships
