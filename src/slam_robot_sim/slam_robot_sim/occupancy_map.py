"""2D log-odds occupancy grid mapping with Bresenham raytracing."""

# الوظيفة: بناء وتحديث خريطة الإشغال ثنائية الأبعاد من نقاط سحابة المسافة.
# الغرض: فصل خوارزمية تتبع الأشعة وتحديث الاحتمالات لتسهيل اختبارها.

import math


def bresenham_line(x0, y0, x1, y1):
    """Return integer grid coordinates along the line from (x0, y0) to (x1, y1)."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0

    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


class OccupancyMap:
    """Maintain a 2D log-odds grid updated from planar hit points."""

    def __init__(
        self,
        *,
        resolution_m=0.04,
        width=300,
        height=300,
        origin_x=None,
        origin_y=None,
        log_odds_hit=0.85,
        log_odds_miss=-0.40,
        log_odds_min=-2.0,
        log_odds_max=3.5,
    ):
        """Initialize an empty occupancy grid centered near (0, 0) by default."""
        if resolution_m <= 0:
            raise ValueError('resolution_m must be positive')
        if width <= 0 or height <= 0:
            raise ValueError('width and height must be positive')
        if log_odds_hit <= 0 or log_odds_miss >= 0:
            raise ValueError('log_odds_hit must be > 0 and log_odds_miss < 0')
        if log_odds_min >= 0 or log_odds_max <= 0:
            raise ValueError('log_odds_min must be < 0 and log_odds_max > 0')

        self.resolution = float(resolution_m)
        self.width = int(width)
        self.height = int(height)
        self.origin_x = (
            -(self.width * self.resolution) / 2.0
            if origin_x is None else float(origin_x)
        )
        self.origin_y = (
            -(self.height * self.resolution) / 2.0
            if origin_y is None else float(origin_y)
        )
        self.log_odds_hit = float(log_odds_hit)
        self.log_odds_miss = float(log_odds_miss)
        self.log_odds_min = float(log_odds_min)
        self.log_odds_max = float(log_odds_max)

        size = self.width * self.height
        self._log_odds = [0.0] * size
        self._observed = [False] * size

    def clear(self):
        """Reset all cells back to unknown state."""
        size = self.width * self.height
        self._log_odds = [0.0] * size
        self._observed = [False] * size

    def world_to_grid(self, x_m, y_m):
        """Convert world coordinates in meters to (gx, gy) grid indices."""
        gx = int(math.floor((x_m - self.origin_x) / self.resolution))
        gy = int(math.floor((y_m - self.origin_y) / self.resolution))
        return gx, gy

    def in_bounds(self, gx, gy):
        """Return True if (gx, gy) lies inside the grid."""
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _add_log_odds(self, gx, gy, delta):
        idx = gy * self.width + gx
        updated = self._log_odds[idx] + delta
        if updated < self.log_odds_min:
            updated = self.log_odds_min
        elif updated > self.log_odds_max:
            updated = self.log_odds_max
        self._log_odds[idx] = updated
        self._observed[idx] = True

    def update_scan(self, origin_x_m, origin_y_m, hit_points_xy):
        """Trace rays from sensor/robot origin to each hit point in map frame."""
        ox, oy = self.world_to_grid(origin_x_m, origin_y_m)
        if not self.in_bounds(ox, oy):
            return 0

        free_cells = set()
        hit_cells = set()
        for px, py in hit_points_xy:
            hx, hy = self.world_to_grid(px, py)
            ray = bresenham_line(ox, oy, hx, hy)
            for cx, cy in ray[:-1]:
                if not self.in_bounds(cx, cy):
                    break
                free_cells.add((cx, cy))
            if self.in_bounds(hx, hy):
                hit_cells.add((hx, hy))

        # Avoid marking an endpoint cell as free in the same scan.
        free_cells -= hit_cells

        for cx, cy in free_cells:
            self._add_log_odds(cx, cy, self.log_odds_miss)
        for cx, cy in hit_cells:
            self._add_log_odds(cx, cy, self.log_odds_hit)

        return len(hit_cells)

    def to_occupancy_data(self):
        """Return ROS OccupancyGrid integer data (-1 unknown, 0..100 probability)."""
        data = [-1] * (self.width * self.height)
        for idx, seen in enumerate(self._observed):
            if not seen:
                continue
            prob = 1.0 - (1.0 / (1.0 + math.exp(self._log_odds[idx])))
            data[idx] = int(round(prob * 100.0))
        return data
