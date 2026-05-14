# TrackedShip.msg
# Represents a tracked ship (Target Ship) with position, velocity, and collision radius.
# Published on /tracked_ship by the target ship controller.

std_msgs/Header header
geometry_msgs/Pose  pose      # current position + orientation in map frame
geometry_msgs/Twist twist     # current velocity (linear + angular)
float64             radius    # collision radius (meters) for costmap projection
