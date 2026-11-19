"""Tests for planar ToF grid projection."""

# الاختبار: التحقق من تحويل قياسات VL53L5CX إلى نقاط ثنائية الأبعاد.
# الغرض: ضمان صحة الإحداثيات والاتجاه واستبعاد القراءات غير الصالحة.

import math

import pytest

from slam_robot_sim.mapping_utils import project_grid_points


def project(grid, **overrides):
    """Project a grid row using the default three-sensor geometry."""
    options = {
        'sensor_x_m': 0.0,
        'sensor_y_m': 0.0,
        'yaw_rad': 0.0,
        'fov_rad': math.radians(45.0),
        'row': 3,
        'reverse_columns': False,
        'range_min_m': 0.05,
        'range_max_m': 4.0,
    }
    options.update(overrides)
    return project_grid_points(grid, **options)


def test_projects_center_zone_forward_in_meters():
    """A 1 m center-zone reading should project forward with z=0."""
    grid = [-1] * 64
    grid[3 * 8 + 4] = 1000

    points = project(grid)

    assert len(points) == 1
    x, y, z = points[0]
    assert x == pytest.approx(math.cos(math.radians(2.8125)))
    assert y == pytest.approx(math.sin(math.radians(2.8125)))
    assert z == 0.0


def test_applies_sensor_yaw_and_mount_offset():
    """Sensor yaw and 1 cm mounting offset affect point location."""
    grid = [-1] * 64
    grid[3 * 8 + 4] = 1000

    points = project(
        grid,
        sensor_x_m=0.01,
        sensor_y_m=-0.01,
        yaw_rad=math.pi / 2.0,
    )

    x, y, _ = points[0]
    assert x == pytest.approx(0.01 - math.sin(math.radians(2.8125)))
    assert y == pytest.approx(-0.01 + math.cos(math.radians(2.8125)))


def test_discards_invalid_and_out_of_range_readings():
    """Invalid, too-near, and too-far cells do not become points."""
    grid = [-1] * 64
    grid[24] = 20
    grid[25] = 5000

    assert project(grid) == []


def test_rejects_grid_with_wrong_size():
    """Projection requires the full 8-by-8 sensor grid."""
    with pytest.raises(ValueError, match='64'):
        project([1000] * 63)
