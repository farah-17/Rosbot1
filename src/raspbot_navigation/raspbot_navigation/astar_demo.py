"""Try A* in the terminal, no robot and no ROS nodes needed.

    ros2 run raspbot_navigation astar_demo                 # small test maze
    ros2 run raspbot_navigation astar_demo --dijkstra      # same maze, no heuristic
    ros2 run raspbot_navigation astar_demo --four          # only straight steps
    ros2 run raspbot_navigation astar_demo --map /root/raspbot_lidar_ws/maps/my_room.yaml \
        --start 0 0 --goal 2.0 1.5                          # your own map, in metres
"""

import argparse
import time

from raspbot_navigation.astar import astar, path_length, simplify
from raspbot_navigation.grid_map import GridMap

# '#' = wall, '.' = free, S = start, G = goal
MAZE = """
########################################
#S.....................#...............#
#......................#...............#
#.......#######........#.......#.......#
#.............#........#.......#.......#
#.............#................#.......#
#.............#................#.......#
#....##########........#########.......#
#......................#...............#
#......................#..............G#
########################################
"""


class TextGrid:
    """A tiny stand-in for GridMap, made from the text maze above."""

    def __init__(self, text):
        lines = [line for line in text.strip().splitlines()]
        self.height, self.width = len(lines), len(lines[0])
        self.resolution = 1.0
        self.lines = lines
        self.start = self.goal = None
        for row, line in enumerate(lines):
            for col, ch in enumerate(line):
                if ch == 'S':
                    self.start = (col, row)
                if ch == 'G':
                    self.goal = (col, row)

    def is_free(self, col, row):
        return (0 <= col < self.width and 0 <= row < self.height
                and self.lines[row][col] != '#')


def show(grid, path, waypoints):
    on_path, corners = set(path or []), set(waypoints or [])
    for row, line in enumerate(grid.lines):
        out = ''
        for col, ch in enumerate(line):
            if ch in 'SG#':
                out += ch
            elif (col, row) in corners:
                out += 'o'
            elif (col, row) in on_path:
                out += '*'
            else:
                out += ' '
        print(out)


def main():
    parser = argparse.ArgumentParser(description='A* demo')
    parser.add_argument('--dijkstra', action='store_true', help='no heuristic (weight 0)')
    parser.add_argument('--four', action='store_true', help='4 neighbours instead of 8')
    parser.add_argument('--weight', type=float, default=1.0, help='heuristic weight')
    parser.add_argument('--map', help='a map .yaml file instead of the test maze')
    parser.add_argument('--start', type=float, nargs=2, metavar=('X', 'Y'), default=[0.0, 0.0])
    parser.add_argument('--goal', type=float, nargs=2, metavar=('X', 'Y'))
    parser.add_argument('--inflate', type=float, default=0.20, help='inflation radius (m)')
    args = parser.parse_args()
    weight = 0.0 if args.dijkstra else args.weight

    if args.map:
        grid = GridMap(args.map)
        grid.inflate(args.inflate)
        if args.goal is None:
            parser.error('--goal X Y is needed with --map')
        start = grid.world_to_cell(*args.start)
        goal = grid.world_to_cell(*args.goal)
        print(f'map {grid.width} x {grid.height} cells, {grid.resolution} m per cell')
        print(f'start {args.start} m -> cell {start}, goal {args.goal} m -> cell {goal}')
    else:
        grid = TextGrid(MAZE)
        start, goal = grid.start, grid.goal

    t0 = time.perf_counter()
    path, expanded = astar(grid, start, goal, diagonal=not args.four, weight=weight)
    ms = (time.perf_counter() - t0) * 1000.0

    if path is None:
        print(f'NO PATH found. Cells expanded: {expanded}')
        if args.map:
            print('Is the start or goal inside a wall, an unknown area or too close to a wall?')
        return
    waypoints = simplify(grid, path)
    if not args.map:
        show(grid, path, waypoints)
    unit = 'm' if args.map else 'cells'
    print(f'path: {len(path)} cells, length {path_length(grid, path):.2f} {unit}, '
          f'{len(waypoints)} waypoints after simplify')
    print(f'cells expanded: {expanded}   time: {ms:.1f} ms')


if __name__ == '__main__':
    main()
