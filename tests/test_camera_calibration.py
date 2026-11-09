import math

import pytest

from mav.camera_calibration import bottom_row_ground_distance


def test_bottom_row_ground_distance_uses_center_of_last_pixel():
    result = bottom_row_ground_distance(0.25, 35.0, 50.0, 360)
    angle = math.radians(35.0 + 25.0 - 25.0 / 360.0)
    assert result == pytest.approx(0.25 / math.tan(angle))


def test_bottom_row_ground_distance_rejects_invalid_geometry():
    with pytest.raises(ValueError, match="height_m"):
        bottom_row_ground_distance(0.0, 35.0, 50.0, 360)
    with pytest.raises(ValueError, match="vertical_fov_deg"):
        bottom_row_ground_distance(0.25, 35.0, 180.0, 360)
    with pytest.raises(ValueError, match="frame_height_px"):
        bottom_row_ground_distance(0.25, 35.0, 50.0, 0)


def test_bottom_row_ground_distance_rejects_ray_not_intersecting_floor_ahead():
    with pytest.raises(ValueError, match="intersect the floor"):
        bottom_row_ground_distance(0.25, -30.0, 20.0, 360)
