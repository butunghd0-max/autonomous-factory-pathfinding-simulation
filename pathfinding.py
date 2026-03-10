"""
pathfinding.py -- A* and Dijkstra with weighted terrain, congestion, and
optional explored-nodes tracking for visualization and comparison.
"""

import heapq
import random as _random
from config import Algorithm, CONGESTION_WEIGHT


def _heuristic(a: tuple, b: tuple) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(floor, start: tuple, end: tuple,
              algorithm: Algorithm = Algorithm.ASTAR,
              congestion_map: dict | None = None,
              jitter: float = 0.0,
              track_explored: bool = False):
    """
    Find the least-cost path from *start* to *end*.

    Returns (path, explored_set) if track_explored, else just path.
    Jitter adds random noise to edge costs for deadlock rerouting.
    """
    empty_result = ([], set()) if track_explored else []

    if not floor.is_walkable(start[0], start[1]):
        return empty_result
    if not floor.is_walkable(end[0], end[1]):
        return empty_result

    use_heuristic = algorithm == Algorithm.ASTAR

    counter = 0
    open_heap = []
    heapq.heappush(open_heap, (0, counter, start))

    came_from: dict[tuple, tuple | None] = {start: None}
    g_cost: dict[tuple, float] = {start: 0}
    explored: set[tuple] = set()

    while open_heap:
        _f, _cnt, current = heapq.heappop(open_heap)

        if current in explored:
            continue
        explored.add(current)

        if current == end:
            path = _reconstruct(came_from, end)
            return (path, explored) if track_explored else path

        for neighbor in floor.get_neighbors(current[0], current[1]):
            edge_cost = floor.cell_cost(neighbor[0], neighbor[1])

            if congestion_map and neighbor in congestion_map:
                edge_cost += congestion_map[neighbor] * CONGESTION_WEIGHT

            if jitter > 0:
                edge_cost += _random.uniform(0, jitter)

            new_g = g_cost[current] + edge_cost

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                came_from[neighbor] = current

                # Skip heap push if we already found the goal
                if neighbor == end:
                    path = _reconstruct(came_from, end)
                    explored.add(neighbor)
                    return (path, explored) if track_explored else path

                h = _heuristic(neighbor, end) if use_heuristic else 0
                f = new_g + h
                counter += 1
                heapq.heappush(open_heap, (f, counter, neighbor))

    return empty_result


def compare_algorithms(floor, start, end, congestion_map=None):
    astar_path, astar_explored = find_path(
        floor, start, end, Algorithm.ASTAR, congestion_map,
        track_explored=True)
    dijkstra_path, dijkstra_explored = find_path(
        floor, start, end, Algorithm.DIJKSTRA, congestion_map,
        track_explored=True)
    return {
        "astar_path": astar_path,
        "astar_explored": astar_explored,
        "astar_nodes": len(astar_explored),
        "dijkstra_path": dijkstra_path,
        "dijkstra_explored": dijkstra_explored,
        "dijkstra_nodes": len(dijkstra_explored),
    }


def _reconstruct(came_from: dict, current: tuple) -> list:
    path = [current]
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
