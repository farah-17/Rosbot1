"""A* path planning on a GridMap. No ROS needed.

    path, expanded = astar(grid, (col1, row1), (col2, row2))

path is a list of (col, row) cells from start to goal, or None if the goal
cannot be reached. expanded is how many cells A* had to look at.
"""

import heapq
import math

# The 8 neighbours of a cell: (d_col, d_row, cost). Straight steps cost 1,
# diagonal steps cost sqrt(2) = 1.414 (they are longer).
STEPS_4 = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
STEPS_8 = STEPS_4 + [(1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                     (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def heuristic(a, b, diagonal=True):
    """Estimate of the cost from cell a to cell b. Never too big, so A*
    still finds the shortest path."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    if diagonal:   # "octile" distance: diagonal steps first, then straight
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
    return dx + dy  # "Manhattan" distance: only straight steps allowed


def astar(grid, start, goal, diagonal=True, weight=1.0):
    """Find the cheapest path from start to goal.

    weight = 1.0 is normal A*. weight = 0.0 turns it into Dijkstra's
    algorithm (no heuristic). weight > 1 is faster but may not be shortest.
    """
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return None, 0
    steps = STEPS_8 if diagonal else STEPS_4

    # The "open list": cells to look at next, sorted by f = g + h
    open_heap = [(0.0, 0.0, start)]          # (f, g, cell)
    came_from = {start: None}                 # how we reached each cell
    g_cost = {start: 0.0}                     # cheapest known cost from start
    expanded = 0

    while open_heap:
        f, g, cell = heapq.heappop(open_heap)
        if g > g_cost[cell]:
            continue                          # an old, worse entry: skip it
        expanded += 1
        if cell == goal:
            return rebuild_path(came_from, goal), expanded

        col, row = cell
        for dc, dr, step_cost in steps:
            nxt = (col + dc, row + dr)
            if not grid.is_free(*nxt):
                continue
            # Do not cut corners: a diagonal step needs both side cells free
            if dc and dr and not (grid.is_free(col + dc, row) and grid.is_free(col, row + dr)):
                continue
            new_g = g + step_cost
            if new_g < g_cost.get(nxt, math.inf):
                g_cost[nxt] = new_g
                came_from[nxt] = cell
                h = heuristic(nxt, goal, diagonal)
                heapq.heappush(open_heap, (new_g + weight * h, new_g, nxt))

    return None, expanded                     # open list empty: no path


def rebuild_path(came_from, cell):
    """Walk backwards from the goal to the start, then reverse."""
    path = []
    while cell is not None:
        path.append(cell)
        cell = came_from[cell]
    path.reverse()
    return path


def line_is_free(grid, a, b):
    """True if the straight line between cells a and b only crosses free cells."""
    (c0, r0), (c1, r1) = a, b
    n = max(abs(c1 - c0), abs(r1 - r0)) * 2 + 1   # sample twice per cell
    for i in range(n + 1):
        t = i / n
        c = int(round(c0 + (c1 - c0) * t))
        r = int(round(r0 + (r1 - r0) * t))
        if not grid.is_free(c, r):
            return False
    return True


def simplify(grid, path):
    """Shorten the zig-zag cell path to a few straight lines (waypoints).
    From each waypoint, jump to the furthest later cell that can be reached
    in a straight line without touching a blocked cell."""
    if not path or len(path) < 3:
        return list(path or [])
    waypoints = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_is_free(grid, path[i], path[j]):
            j -= 1
        waypoints.append(path[j])
        i = j
    return waypoints


def path_length(grid, cells):
    """Length of a cell path in metres."""
    total = 0.0
    for a, b in zip(cells, cells[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])
    return total * grid.resolution
