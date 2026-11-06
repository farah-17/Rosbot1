"""Live obstacles seen by the lidar, kept in memory on top of the saved map.

No ROS needed: this file only uses plain Python, so you can test it anywhere.

The saved map (.pgm) is never changed. Every lidar scan is turned into map
cells; a cell becomes a "live obstacle" once it has been hit in a few scans
(this filters single noisy points) and stays one for `ttl` seconds after the
last hit, so an obstacle that is removed disappears by itself.

    layer = LiveObstacleLayer(grid)
    layer.update(now, ranges, angle_min, angle_increment, range_min, range_max,
                 laser=(x, y, yaw in base_link), robot=(x, y, yaw in map))
    layer.apply(now, robot_xy)      # puts the obstacles into grid.dynamic
"""

import math
import threading


def scan_points_in_map(ranges, angle_min, angle_increment, low, high, laser, robot, stride=1):
    """(x, y) in the map frame of every `stride`-th lidar point with low < range < high (metres).
    laser = (x, y, yaw) of the lidar in base_link, robot = (x, y, yaw) in the map.
    stride > 1 saves CPU: the Pi has to run Cartographer too."""
    lx0, ly0, lyaw = laser
    rx, ry, ryaw = robot
    cos_l, sin_l = math.cos(lyaw), math.sin(lyaw)
    cos_r, sin_r = math.cos(ryaw), math.sin(ryaw)
    points = []
    for i in range(0, len(ranges), stride):
        r = ranges[i]
        if not (low < r < high):            # also skips nan and inf
            continue
        a = angle_min + i * angle_increment
        px, py = r * math.cos(a), r * math.sin(a)          # in the lidar frame
        bx = lx0 + px * cos_l - py * sin_l                  # in base_link
        by = ly0 + px * sin_l + py * cos_l
        points.append((rx + bx * cos_r - by * sin_r,        # in the map
                       ry + bx * sin_r + by * cos_r))
    return points


class LiveObstacleLayer:

    def __init__(self, grid, min_range=0.12, max_range=2.0, ttl=3.0, min_hits=2,
                 inflation=0.15, keep_clear=0.12, stride=3):
        self.grid = grid
        self.min_range = min_range      # m: ignore lidar points closer than this
        self.max_range = max_range      # m: ignore points further than this
        self.ttl = ttl                  # s: an obstacle disappears this long after its last hit
        self.min_hits = min_hits        # scans a cell must be hit in before it counts
        self.inflation = inflation      # m kept free around a live obstacle
        self.keep_clear = keep_clear    # m around the robot that is never blocked
        self.stride = stride            # use every n-th lidar point (2000 points per scan is a lot)
        self.seen = {}                  # (col, row) -> [time of last hit, number of hits]
        self.lock = threading.Lock()

    def update(self, now, ranges, angle_min, angle_increment, range_min, range_max,
               laser, robot):
        """Add the points of one scan. laser = lidar pose in base_link, robot = pose in map."""
        low = max(range_min, self.min_range)
        high = min(range_max, self.max_range)
        grid = self.grid
        hit = set()
        for mx, my in scan_points_in_map(ranges, angle_min, angle_increment, low, high,
                                         laser, robot, self.stride):
            col, row = grid.world_to_cell(mx, my)
            # Walls and unknown areas are already blocked in the saved map.
            if grid.inside(col, row) and not grid.blocked[row][col]:
                hit.add((col, row))
        with self.lock:
            for cell in hit:
                entry = self.seen.get(cell)
                if entry is None or now - entry[0] > self.ttl:
                    self.seen[cell] = [now, 1]
                else:
                    entry[0] = now
                    entry[1] += 1
            for cell in [c for c, e in self.seen.items() if now - e[0] > self.ttl]:
                del self.seen[cell]

    def obstacle_cells(self, now):
        """Cells that count as obstacles right now."""
        with self.lock:
            return [c for c, e in self.seen.items()
                    if e[1] >= self.min_hits and now - e[0] <= self.ttl]

    def apply(self, now, robot_xy=None):
        """Copy the current obstacles (made bigger by `inflation`) into the grid.
        Returns how many obstacle cells there are."""
        cells = self.obstacle_cells(now)
        clear = None
        if robot_xy is not None:
            clear = (self.grid.world_to_cell(*robot_xy), self.keep_clear)
        self.grid.set_live_obstacles(cells, self.inflation, clear)
        return len(cells)

    def clear(self):
        with self.lock:
            self.seen.clear()
        self.grid.clear_live_obstacles()
