"""Load a saved map (.yaml + .pgm) into a grid of cells that A* can search.

No ROS needed: this file only uses plain Python, so you can test it anywhere.

    grid = GridMap('/root/raspbot_lidar_ws/maps/my_room.yaml')
    grid.inflate(0.20)                  # keep 20 cm away from walls
    cell = grid.world_to_cell(1.0, 0.5) # metres -> (col, row)
    grid.is_free(*cell)

Cells are stored as grid.cells[row][col], with row 0 at the BOTTOM of the map
(small y), the same way round as the 'map' frame in ROS.

Two layers decide what is free:
  * the SAVED map (grid.blocked): loaded once, never changed;
  * the LIVE layer (grid.dynamic): obstacles the lidar sees right now, kept in
    memory only (see live_layer.py). The .pgm file is never written.
"""

import math
import os

import yaml

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def read_pgm(path):
    """Return (width, height, max_value, pixels) of a .pgm picture (P5 or P2)."""
    with open(path, 'rb') as f:
        data = f.read()
    # The header is 4 words: magic, width, height, max value. Lines with # are comments.
    words, pos = [], 0
    while len(words) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        words.append(data[start:pos].decode())
    magic, width, height, max_value = words[0], int(words[1]), int(words[2]), int(words[3])
    if magic == 'P5':
        pixels = data[pos + 1:pos + 1 + width * height]
    elif magic == 'P2':
        pixels = [int(v) for v in data[pos:].split()]
    else:
        raise ValueError(f'{path}: not a PGM picture (starts with {magic})')
    return width, height, max_value, pixels


def disk_offsets(radius_cells):
    """All the (d_col, d_row) offsets inside a circle of radius_cells cells."""
    r = radius_cells
    return [(dc, dr) for dr in range(-r, r + 1) for dc in range(-r, r + 1)
            if dc * dc + dr * dr <= r * r]


class GridMap:

    def __init__(self, yaml_path, unknown_is_free=False):
        with open(yaml_path) as f:
            info = yaml.safe_load(f)
        image = info['image']
        if not os.path.isabs(image):           # path is relative to the yaml file
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)

        self.resolution = float(info['resolution'])     # metres per cell
        self.origin_x = float(info['origin'][0])        # world position of the
        self.origin_y = float(info['origin'][1])        # bottom-left cell
        negate = int(info.get('negate', 0))
        occupied_thresh = float(info.get('occupied_thresh', 0.65))
        free_thresh = float(info.get('free_thresh', 0.25))

        self.width, self.height, max_value, pixels = read_pgm(image)

        # Pixel -> FREE / OCCUPIED / UNKNOWN, the same rule map_server uses.
        # The picture's first line is the TOP of the map, so we flip it.
        self.cells = []
        for row in range(self.height):
            line = self.height - 1 - row
            cells_row = []
            for col in range(self.width):
                value = pixels[line * self.width + col]
                p = value / max_value if negate else (max_value - value) / max_value
                if value == 205 and max_value == 255:
                    # map_saver paints unknown cells grey 205. With the usual
                    # free_thresh 0.25 the rule below would call them free.
                    cells_row.append(UNKNOWN)
                elif p > occupied_thresh:
                    cells_row.append(OCCUPIED)
                elif p < free_thresh:
                    cells_row.append(FREE)
                else:
                    cells_row.append(UNKNOWN)
            self.cells.append(cells_row)

        # blocked[row][col] is what the planner uses. inflate() makes it bigger.
        self.unknown_is_free = unknown_is_free
        self.inflation_radius = 0.0
        self.blocked = [[self._raw_blocked(c) for c in r] for r in self.cells]
        # Live obstacles from the lidar (set of (col, row)); see set_live_obstacles().
        self.dynamic = set()

    def _raw_blocked(self, value):
        if value == OCCUPIED:
            return True
        return value == UNKNOWN and not self.unknown_is_free

    # ---- converting between metres and cells --------------------------------
    def world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return col, row

    def cell_to_world(self, col, row):
        """Centre of the cell, in metres."""
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def inside(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def is_static_free(self, col, row):
        """Free in the SAVED map (ignores live obstacles)."""
        return self.inside(col, row) and not self.blocked[row][col]

    def is_free(self, col, row):
        """Free in the saved map AND not covered by a live obstacle."""
        return self.is_static_free(col, row) and (col, row) not in self.dynamic

    def is_wall(self, col, row):
        """True for a real wall or anything outside the map (for the simulator)."""
        return not self.inside(col, row) or self.cells[row][col] == OCCUPIED

    # ---- making walls "fatter" so the robot keeps its distance --------------
    def inflate(self, radius_m):
        """Block every cell closer than radius_m to a wall (or unknown area)."""
        self.inflation_radius = radius_m
        disk = disk_offsets(int(math.ceil(radius_m / self.resolution)))
        self.blocked = [[self._raw_blocked(c) for c in row] for row in self.cells]
        for row in range(self.height):
            for col in range(self.width):
                if not self._raw_blocked(self.cells[row][col]):
                    continue
                # Only walls that touch free space need to grow
                if not self._touches_free(col, row):
                    continue
                for dc, dr in disk:
                    c, rr = col + dc, row + dr
                    if self.inside(c, rr):
                        self.blocked[rr][c] = True

    def _touches_free(self, col, row):
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            c, r = col + dc, row + dr
            if self.inside(c, r) and not self._raw_blocked(self.cells[r][c]):
                return True
        return False

    # ---- live obstacles (memory only, the saved map is untouched) ---------------
    def set_live_obstacles(self, cells, radius_m, keep_clear=None):
        """Replace the live layer: block every cell within radius_m of `cells`.
        keep_clear = ((col, row), metres): never block cells that close to the
        robot, so it can always find a way out."""
        disk = disk_offsets(int(math.ceil(radius_m / self.resolution)))
        live = set()
        for col, row in cells:
            for dc, dr in disk:
                live.add((col + dc, row + dr))
        if keep_clear is not None:
            (rc, rr), metres = keep_clear
            r = int(math.ceil(metres / self.resolution))
            live -= {(rc + dc, rr + dr) for dc, dr in disk_offsets(r)}
        self.dynamic = live          # one assignment, so other threads never see half of it

    def clear_live_obstacles(self):
        self.dynamic = set()

    def live_on_line(self, a, b):
        """True if the straight line between cells a and b crosses a live obstacle."""
        dynamic = self.dynamic
        if not dynamic:
            return False
        (c0, r0), (c1, r1) = a, b
        n = max(abs(c1 - c0), abs(r1 - r0)) * 2 + 1   # sample twice per cell
        for i in range(n + 1):
            t = i / n
            if (int(round(c0 + (c1 - c0) * t)), int(round(r0 + (r1 - r0) * t))) in dynamic:
                return True
        return False

    def nearest_free(self, col, row, max_cells=10, live=True):
        """The closest free cell to (col, row), or None. Used when the robot or
        the goal is a little too close to a wall. live=False ignores live obstacles."""
        free = self.is_free if live else self.is_static_free
        if free(col, row):
            return col, row
        for radius in range(1, max_cells + 1):
            best = None
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if max(abs(dc), abs(dr)) != radius:
                        continue                     # only the ring at this radius
                    if free(col + dc, row + dr):
                        d = dc * dc + dr * dr
                        if best is None or d < best[0]:
                            best = (d, col + dc, row + dr)
            if best:
                return best[1], best[2]
        return None
