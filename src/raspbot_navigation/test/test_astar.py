"""Tests for the A* planner and the map loader. Run inside the container:

    cd /root/raspbot_lidar_ws/src/raspbot_navigation
    python3 -m pytest -v test
"""

import os

from raspbot_navigation.astar import astar, heuristic, path_length, simplify
from raspbot_navigation.grid_map import GridMap


class Grid:
    """A tiny hand-made map: '#' = wall, '.' = free. Row 0 is the first line."""

    def __init__(self, *lines):
        self.lines, self.resolution = lines, 1.0

    def is_free(self, col, row):
        return 0 <= row < len(self.lines) and 0 <= col < len(self.lines[row]) \
            and self.lines[row][col] == '.'


def test_straight_line():
    grid = Grid('.....')
    path, _ = astar(grid, (0, 0), (4, 0))
    assert path == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]


def test_goes_around_a_wall():
    grid = Grid('.#...',
                '.#.#.',
                '...#.')
    path, _ = astar(grid, (0, 0), (4, 0))
    assert path[0] == (0, 0) and path[-1] == (4, 0)
    assert all(grid.is_free(*cell) for cell in path)


def test_no_path_when_walled_in():
    grid = Grid('..#..',
                '..#..')
    path, expanded = astar(grid, (0, 0), (4, 0))
    assert path is None
    assert expanded > 0


def test_goal_in_wall_is_refused():
    grid = Grid('..#')
    assert astar(grid, (0, 0), (2, 0)) == (None, 0)


def test_no_corner_cutting():
    grid = Grid('.#',
                '..')
    path, _ = astar(grid, (0, 0), (1, 1))
    assert path == [(0, 0), (0, 1), (1, 1)]   # not the diagonal past the wall


def test_dijkstra_same_length_but_more_work():
    grid = Grid(*['.' * 30] * 30)
    a_path, a_expanded = astar(grid, (0, 0), (29, 20))
    d_path, d_expanded = astar(grid, (0, 0), (29, 20), weight=0.0)
    assert abs(path_length(grid, a_path) - path_length(grid, d_path)) < 1e-9
    assert a_expanded < d_expanded


def test_heuristic_never_overestimates():
    # straight 3 and diagonal 4: true cost = 4 * 1.414 + 3
    assert abs(heuristic((0, 0), (7, 4)) - (4 * 2 ** 0.5 + 3)) < 1e-9
    assert heuristic((0, 0), (7, 4), diagonal=False) == 11


def test_simplify_keeps_ends_and_only_free_lines():
    grid = Grid('......',
                '.####.',
                '......')
    path, _ = astar(grid, (0, 0), (5, 2))
    waypoints = simplify(grid, path)
    assert waypoints[0] == (0, 0) and waypoints[-1] == (5, 2)
    assert len(waypoints) < len(path)


def test_grid_map_loads_pgm(tmp_path):
    # 4 x 3 picture: top line all walls (0), the rest free (254), one unknown (205)
    pixels = bytes([0, 0, 0, 0,
                    254, 254, 254, 254,
                    254, 205, 254, 254])
    (tmp_path / 'room.pgm').write_bytes(b'P5\n# test\n4 3\n255\n' + pixels)
    (tmp_path / 'room.yaml').write_text(
        'image: room.pgm\nresolution: 0.5\norigin: [-1.0, -1.0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    grid = GridMap(os.path.join(tmp_path, 'room.yaml'))
    assert (grid.width, grid.height) == (4, 3)
    assert grid.is_free(0, 0)              # bottom-left of the picture
    assert not grid.is_free(1, 0)          # the unknown (grey) pixel
    assert not grid.is_free(0, 2)          # the top line is a wall
    assert grid.world_to_cell(-0.9, -0.9) == (0, 0)
    assert grid.cell_to_world(0, 0) == (-0.75, -0.75)
    grid.inflate(0.5)                      # one cell
    assert not grid.is_free(0, 1)          # next to the wall: now blocked
