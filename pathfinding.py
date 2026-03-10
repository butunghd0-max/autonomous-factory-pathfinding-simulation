"""
pathfinding.py — A* and Dijkstra implementations for grid-based navigation.
"""

import heapq
from config import Algorithm


def _heuristic(a: tuple, b: tuple) -> int:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(floor, start: tuple, end: tuple,
              algorithm: Algorithm = Algorithm.ASTAR):
    """
    Find the shortest path on *floor* from *start* to *end*.

    Parameters
    ----------
    floor : FactoryFloor
        The environment instance.
    start : (row, col)
    end   : (row, col)
    algorithm : Algorithm enum member

    Returns
    -------
    list[(row, col)] — ordered path from start to end, **including both
    endpoints**, or an empty list if no path exists.
    """
    if not floor.is_walkable(start[0], start[1]):
        return []
    if not floor.is_walkable(end[0], end[1]):
        return []

    use_heuristic = algorithm == Algorithm.ASTAR

    # Each entry: (f_cost, counter, (row, col))
    counter = 0
    open_heap = []
    heapq.heappush(open_heap, (0, counter, start))

    came_from: dict[tuple, tuple | None] = {start: None}
    g_cost: dict[tuple, float] = {start: 0}

    while open_heap:
        _f, _cnt, current = heapq.heappop(open_heap)

        if current == end:
            return _reconstruct(came_from, end)

        for neighbor in floor.get_neighbors(current[0], current[1]):
            new_g = g_cost[current] + 1          # uniform edge weight

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                h = _heuristic(neighbor, end) if use_heuristic else 0
                f = new_g + h
                counter += 1
                heapq.heappush(open_heap, (f, counter, neighbor))
                came_from[neighbor] = current

    return []   # no path


def _reconstruct(came_from: dict, current: tuple) -> list:
    """Walk the parent map backward to reconstruct the full path."""
    path = [current]
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
