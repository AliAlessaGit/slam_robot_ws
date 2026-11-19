"""Geometry helpers for converting VL53L5CX grids to planar XYZ points."""

# الوظيفة: تحويل الصف الأوسط من شبكات VL53L5CX إلى نقاط ثلاثية الأبعاد مستوية.
# الغرض: عزل الإسقاط الهندسي وحسابات الزوايا لتسهيل اختبارها وضبطها.

import math


def project_grid_points(
        grid, sensor_x_m, sensor_y_m, yaw_rad, fov_rad, row,
        reverse_columns, range_min_m, range_max_m):
    """Project one ToF grid row into planar XYZ points in base coordinates."""
    if len(grid) != 64:
        raise ValueError('grid must contain exactly 64 values')
    if not 0 <= row < 8:
        raise ValueError('row must be between 0 and 7')
    if not 0 < fov_rad <= math.pi:
        raise ValueError('horizontal FOV must be in (0, pi]')
    if range_min_m <= 0 or range_max_m <= range_min_m:
        raise ValueError('range limits must be positive and ordered')

    points = []
    angle_increment = fov_rad / 8.0
    for column in range(8):
        distance_mm = grid[row * 8 + column]
        if distance_mm <= 0:
            continue
        distance_m = distance_mm / 1000.0
        if not range_min_m <= distance_m <= range_max_m:
            continue
        angle_column = 7 - column if reverse_columns else column
        angle = yaw_rad + (angle_column - 3.5) * angle_increment
        x = sensor_x_m + distance_m * math.cos(angle)
        y = sensor_y_m + distance_m * math.sin(angle)
        points.append((x, y, 0.0))
    return points


def yaw_from_quaternion(quaternion):
    """Return planar yaw in radians from a ROS quaternion-like object."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_difference(angle, reference):
    """Return the signed, wrapped difference between two angles."""
    return math.atan2(math.sin(angle - reference), math.cos(angle - reference))
