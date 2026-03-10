"""
robot.py — AGV (Automated Guided Vehicle) and FleetManager classes.
"""

import random
from config import (
    Algorithm, ROBOT_COLORS, INITIAL_ROBOT_COUNT, AUTO_DISPATCH_INTERVAL,
    STATION_LOAD, STATION_DELIVER, TRAIL_MAX_LENGTH,
)
from pathfinding import find_path


class AGV:
    """A single autonomous guided vehicle navigating the factory floor."""

    _id_counter = 0

    def __init__(self, position: tuple, color: tuple | None = None):
        AGV._id_counter += 1
        self.id = AGV._id_counter
        self.position = position       # (row, col)
        self.destination = None        # (row, col) or None
        self.path: list[tuple] = []    # remaining path (current pos removed)
        self.status = "idle"           # idle | moving | blocked | recalculating
        self.color = color or ROBOT_COLORS[(self.id - 1) % len(ROBOT_COLORS)]
        self.tasks_completed = 0
        self.total_steps = 0
        self.recalculations = 0
        self.trail_history: list[tuple] = []   # recent positions for trail rendering

    # ── task management ──────────────────────────────────────────────────
    def assign_task(self, destination: tuple, floor, algorithm: Algorithm):
        """Compute a path to *destination* and begin moving."""
        self.destination = destination
        self.path = find_path(floor, self.position, destination, algorithm)
        if self.path:
            self.path.pop(0)   # remove current position
            self.status = "moving"
        else:
            self.status = "blocked"

    def recalculate(self, floor, algorithm: Algorithm):
        """Re-run pathfinding from current position (dynamic obstacle appeared)."""
        self.recalculations += 1
        if self.destination is None:
            self.status = "idle"
            return
        self.path = find_path(floor, self.position, self.destination, algorithm)
        if self.path:
            self.path.pop(0)
            self.status = "moving"
        else:
            self.status = "blocked"

    # ── stepping ─────────────────────────────────────────────────────────
    def step(self, floor, occupied_next: set, algorithm: Algorithm):
        """Advance one cell along the path. Returns the *new* position.

        *occupied_next* contains positions already claimed this tick.
        If the next cell is occupied or unwalkable, the robot waits or recalculates.
        """
        if self.status == "idle" or not self.path:
            if self.destination and self.position == self.destination:
                self.tasks_completed += 1
                self.destination = None
                self.path = []
                self.status = "idle"
            return self.position

        next_pos = self.path[0]

        # Blocked by another robot → wait
        if next_pos in occupied_next:
            self.status = "blocked"
            return self.position

        # Blocked by a new obstacle → recalculate
        if not floor.is_walkable(next_pos[0], next_pos[1]):
            self.status = "recalculating"
            self.recalculate(floor, algorithm)
            return self.position

        # Move forward
        self.trail_history.append(self.position)
        if len(self.trail_history) > TRAIL_MAX_LENGTH:
            self.trail_history.pop(0)
        self.position = next_pos
        self.path.pop(0)
        self.total_steps += 1
        self.status = "moving"

        # Arrived?
        if not self.path:
            if self.destination and self.position == self.destination:
                self.tasks_completed += 1
                self.destination = None
                self.status = "idle"

        return self.position


class FleetManager:
    """Coordinates a fleet of AGVs on the factory floor."""

    def __init__(self, floor, algorithm: Algorithm = Algorithm.ASTAR):
        self.floor = floor
        self.algorithm = algorithm
        self.robots: list[AGV] = []
        self._dispatch_timer = 0

    # ── fleet setup ──────────────────────────────────────────────────────
    def spawn_initial(self, count: int = INITIAL_ROBOT_COUNT):
        """Place *count* robots at loading dock stations."""
        load_stations = self.floor.get_stations(STATION_LOAD)
        for i in range(min(count, len(load_stations))):
            agv = AGV(position=load_stations[i])
            self.robots.append(agv)

    def add_robot(self, position: tuple):
        """Add a new robot at the given position."""
        if self.floor.is_walkable(position[0], position[1]):
            agv = AGV(position=position)
            self.robots.append(agv)
            return agv
        return None

    # ── dispatching ──────────────────────────────────────────────────────
    def auto_dispatch(self):
        """Assign idle robots to random delivery (or load) stations."""
        deliver_stations = self.floor.get_stations(STATION_DELIVER)
        load_stations = self.floor.get_stations(STATION_LOAD)
        if not deliver_stations or not load_stations:
            return

        for robot in self.robots:
            if robot.status == "idle" and robot.destination is None:
                # Alternate: if at a load station → go to delivery, else → go to load
                at_load = robot.position in load_stations
                targets = deliver_stations if at_load else load_stations
                dest = random.choice(targets)
                robot.assign_task(dest, self.floor, self.algorithm)

    # ── step all ─────────────────────────────────────────────────────────
    def step_all(self):
        """Advance every robot by one tick, handling collisions."""
        self._dispatch_timer += 1
        if self._dispatch_timer >= AUTO_DISPATCH_INTERVAL:
            self._dispatch_timer = 0
            self.auto_dispatch()

        # Build set of occupied positions (priority: lower-ID robots first)
        occupied_next: set[tuple] = set()
        for robot in sorted(self.robots, key=lambda r: r.id):
            new_pos = robot.step(self.floor, occupied_next, self.algorithm)
            occupied_next.add(new_pos)

    # ── queries ──────────────────────────────────────────────────────────
    def get_all_paths(self) -> dict[int, list[tuple]]:
        """Return {robot_id: remaining_path} for drawing."""
        return {r.id: r.path for r in self.robots}
