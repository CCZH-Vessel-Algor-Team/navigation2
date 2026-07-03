# VOBarrierLines.msg
# U-shaped virtual barrier of 3 line segments around a target ship,
# specific to the VO-RRT collision avoidance algorithm.
# Other planners may use different barrier geometries.

# Pairs: [p0,p1, p2,p3, p4,p5] = 3 segments:
#   Segment 1: lateral barrier perpendicular to OS→TS bearing
#   Segment 2: longitudinal barrier anti-parallel to OS→TS
#   Segment 3: radial outward ray (substituted by long segment; see ray_length param)
geometry_msgs/Point[] points  # exactly 6 points
