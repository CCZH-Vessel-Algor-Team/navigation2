"""A rotating clear beam must not erase an earlier close obstacle."""
import math
from types import SimpleNamespace as Obj
import pytest
from usv_autonomy.radar import RadarWindow, sector_distance


def sector(rays):
    return Obj(angle_start=0., angle_increment=math.pi, range_min=0., range_max=8.,
               intensities=[Obj(echoes=ray) for ray in rays])


def test_zero_returns_are_valid_but_missing_samples_are_not():
    assert math.isinf(sector_distance(sector([[0.] * 9]), 2.))
    with pytest.raises(ValueError):
        sector_distance(sector([[]]), 2.)


def test_nearest_is_computed_after_sensor_offset_for_all_rays():
    first, second = [0.] * 9, [0.] * 9
    first[5], second[6] = 10., 10.
    transform = Obj(rotation=Obj(x=0., y=0., z=0., w=1.), translation=Obj(x=3., y=0., z=0.))
    assert sector_distance(sector([first, second]), 2., transform) == pytest.approx(3.)


def test_close_return_survives_following_clear_beams_until_expiry():
    window = RadarWindow(1.)
    assert window.observe(2., 0., 0.) == 2.
    assert window.observe(float('inf'), .01, .01) == 2.
    assert window.closest(.9) == 2.
    assert math.isinf(window.closest(1.1))


def test_nonfinite_or_oversized_samples_are_not_clear_observations():
    with pytest.raises(ValueError):
        sector_distance(sector([[float('nan')]]), 2.)
    with pytest.raises(ValueError):
        sector_distance(sector([[0.] * 10]), 2., max_samples=9)


def test_invalid_sensor_transform_is_not_ignored_for_obstacle_geometry():
    ray = [0.] * 9
    ray[5] = 10.
    invalid = Obj(rotation=Obj(x=0., y=0., z=0., w=0.), translation=Obj(x=0., y=0., z=0.))
    with pytest.raises(ValueError):
        sector_distance(sector([ray]), 2., invalid)
