"""Geometry is bound to the saved HOLD anchor, including paused tasks."""
import math

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from usv_autonomy.hold_visualization import hold_markers


def test_hold_rings_use_saved_anchor_and_exact_radii_when_paused():
    task = dict(intent='PAUSE', state='PAUSED', hold_anchor=dict(
        frame='map', position=[12., -7., 0.]),
        spec=dict(mode='HOLD', hold=dict(inner=3., outer=10.)))
    result = hold_markers(task, Time(sec=10))
    assert len(result.markers) == 4
    for marker, radius in zip(result.markers[:2], (3., 10.)):
        assert marker.action == Marker.ADD and marker.type == Marker.LINE_STRIP
        assert marker.header.frame_id == 'map' and marker.header.stamp.sec == 10
        assert marker.pose.position.x == 12. and marker.pose.position.y == -7.
        assert len(marker.points) == 129
        assert all(math.isclose(math.hypot(p.x, p.y), radius) for p in marker.points)
        assert math.isclose(marker.points[0].x, marker.points[-1].x)
        assert math.isclose(marker.points[0].y, marker.points[-1].y, abs_tol=1e-12)
    assert 'PAUSE' in result.markers[3].text
    assert result.markers[0].color.g > result.markers[0].color.r
    assert result.markers[1].color.r > result.markers[1].color.g
    task['spec']['mode'] = 'NAVIGATION'
    assert all(m.action == Marker.DELETE for m in hold_markers(task, Time()).markers)
    task['spec']['mode'], task['intent'] = 'HOLD', 'END'
    assert all(m.action == Marker.DELETE for m in hold_markers(task, Time()).markers)
    assert all(m.action == Marker.DELETE for m in hold_markers(None, Time()).markers)
