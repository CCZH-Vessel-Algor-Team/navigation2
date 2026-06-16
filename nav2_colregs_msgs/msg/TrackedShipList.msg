# TrackedShipList.msg
# Batch of tracked ships published by a fusion node.
# Individual ship poses are in the coordinate frame given by header.frame_id.

std_msgs/Header header
TrackedShip[] ships
