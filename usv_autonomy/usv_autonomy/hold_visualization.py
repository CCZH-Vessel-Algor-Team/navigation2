"""Display the selected HOLD task's actual checkpoint, never a UI draft."""
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def hold_markers(task, stamp):
    markers = MarkerArray()
    visible = (task and task['spec']['mode'] == 'HOLD' and task.get('hold_anchor')
               and task['intent'] != 'END' and task['state'] not in ('COMPLETED', 'CANCELED'))
    anchor = task['hold_anchor'] if visible else None
    for identity in range(4):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = anchor['frame'] if anchor else 'map'
        marker.ns, marker.id = 'usv_hold_region', identity
        marker.action = Marker.ADD if visible else Marker.DELETE
        marker.pose.orientation.w = 1.0
        if visible:
            x, y, z = anchor['position']
            marker.pose.position = Point(x=float(x), y=float(y), z=float(z) + .4)
            marker.lifetime.nanosec = 600000000
            marker.color.a = .95
            if identity < 2:
                marker.type = Marker.LINE_STRIP
                marker.scale.x = .10
                radius = task['spec']['hold']['inner' if identity == 0 else 'outer']
                marker.color.r, marker.color.g, marker.color.b = (
                    (.1, 1., .3) if identity == 0 else (1., .55, .05))
                marker.points = [Point(x=radius * math.cos(2 * math.pi * i / 128),
                                       y=radius * math.sin(2 * math.pi * i / 128), z=0.)
                                 for i in range(129)]
            elif identity == 2:
                marker.type = Marker.SPHERE
                marker.scale.x = marker.scale.y = marker.scale.z = .35
                marker.color.r, marker.color.g, marker.color.b = .2, .8, 1.
            else:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.pose.position.z += 1.0
                marker.scale.z = .65
                marker.color.r = marker.color.g = marker.color.b = 1.
                options = task['spec']['hold']
                marker.text = (f'HOLD {task["intent"]} | inner {options["inner"]:g} m'
                               f' | outer {options["outer"]:g} m')
        markers.markers.append(marker)
    return markers
