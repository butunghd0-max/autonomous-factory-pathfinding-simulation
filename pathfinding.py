"""
pathfinding.py -- A* and Dijkstra with weighted terrain and congestion support.
"""

import heapq
from config import Algorithm, CONGESTION_WEIGHT


def _heuristic(a: tuple, b: tuple) -> int:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(floor, start: tuple, end: tuple,
              algorithm: Algorithm = Algorithm.ASTAR,
              congestion_map: dict | None = None):
    """
    Find the shortest path on *floor* from *start* to *end*.

    Parameters
    ----------
    floor : FactoryFloor
    start, end : (row, col)
    algorithm : Algorithm enum
    congestion_map : optional dict[(r,c)] -> int count of nearby robots

    Returns
    -------
    list[(row, col)] including both endpoints, or empty list if no path.
    """
    if not floor.is_walkable(start[0], start[1]):
        return []
    if not floor.is_walkable(end[0], end[1]):
        return []

    use_heuristic = algorithm == Algorithm.ASTAR

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
            # Weighted edge cost
            edge_cost = floor.cell_cost(neighbor[0], neighbor[1])

            # Congestion penalty
            if congestion_map and neighbor in congestion_map:
                edge_cost += congestion_map[neighbor] * CONGESTION_WEIGHT

            new_g = g_cost[current] + edge_cost

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                h = _heuristic(neighbor, end) if use_heuristic else 0
                f = new_g + h
                counter += 1
                heapq.heappush(open_heap, (f, counter, neighbor))
                came_from[neighbor] = current

    return []


def _reconstruct(came_from: dict, current: tuple) -> list:
    """Walk the parent map backward to reconstruct the full path."""
    path = [current]
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
