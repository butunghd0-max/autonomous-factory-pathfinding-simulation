"""
robot.py -- AGV with battery system and FleetManager with congestion-aware routing.
"""

import random
from config import (
    Algorithm, ROBOT_COLORS, INITIAL_ROBOT_COUNT, AUTO_DISPATCH_INTERVAL,
    STATION_LOAD, STATION_DELIVER, CHARGE_STATION, SLOW_ZONE,
    TRAIL_MAX_LENGTH, CONGESTION_RADIUS,
    BATTERY_MAX, BATTERY_DRAIN_PER_STEP, BATTERY_DRAIN_SLOW_ZONE,
    BATTERY_CHARGE_RATE, BATTERY_LOW_THRESHOLD,
)
from pathfinding import find_path


class AGV:
    """A single autonomous guided vehicle with battery management."""

    _id_counter = 0

    def __init__(self, position: tuple, color: tuple | None = None):
        AGV._id_counter += 1
        self.id = AGV._id_counter
        self.position = position
        self.destination = None
        self.path: list[tuple] = []
        self.status = "idle"  # idle | moving | blocked | recalculating | charging
        self.color = color or ROBOT_COLORS[(self.id - 1) % len(ROBOT_COLORS)]
        self.tasks_completed = 0
        self.total_steps = 0
        self.recalculations = 0
        self.trail_history: list[tuple] = []
        # Battery
        self.battery = BATTERY_MAX
        self.is_charging = False
        self._returning_to_charge = False

    # -- task management --------------------------------------------------
    def assign_task(self, destination, floor, algorithm, congestion_map=None):
        """Compute path to destination and start moving."""
        self.destination = destination
        self._returning_to_charge = False
        self.path = find_path(floor, self.position, destination,
                              algorithm, congestion_map)
        if self.path:
            self.path.pop(0)
            self.status = "moving"
        else:
            self.status = "blocked"

    def recalculate(self, floor, algorithm, congestion_map=None):
        """Re-run pathfinding from current position."""
        self.recalculations += 1
        if self.destination is None:
            self.status = "idle"
            return
        self.path = find_path(floor, self.position, self.destination,
                              algorithm, congestion_map)
        if self.path:
            self.path.pop(0)
            self.status = "moving"
        else:
            self.status = "blocked"

    def seek_charger(self, floor, algorithm, congestion_map=None):
        """Find the nearest charging station and go there."""
        chargers = floor.get_stations(CHARGE_STATION)
        if not chargers:
            return
        # Pick the closest charger by Manhattan distance
        chargers.sort(key=lambda c: abs(c[0]-self.position[0]) +
                                     abs(c[1]-self.position[1]))
        for charger in chargers:
            self.destination = charger
            self._returning_to_charge = True
            self.path = find_path(floor, self.position, charger,
                                  algorithm, congestion_map)
            if self.path:
                self.path.pop(0)
                self.status = "moving"
                return
        self.status = "blocked"

    # -- stepping ---------------------------------------------------------
    def step(self, floor, occupied_next: set, algorithm,
             congestion_map=None):
        """Advance one cell. Returns the new position."""

        # Charging logic: if at a charge station and battery < max, charge
        if (self.is_charging and
                floor.grid[self.position[0], self.position[1]] == CHARGE_STATION):
            self.battery = min(BATTERY_MAX, self.battery + BATTERY_CHARGE_RATE)
            self.status = "charging"
            if self.battery >= BATTERY_MAX:
                self.is_charging = False
                self._returning_to_charge = False
                self.status = "idle"
                self.destination = None
                self.path = []
            return self.position

        # Dead battery -- can't move
        if self.battery <= 0:
            self.status = "blocked"
            return self.position

        if self.status in ("idle", "charging") or not self.path:
            if self.destination and self.position == self.destination:
                if self._returning_to_charge:
                    self.is_charging = True
                    self.status = "charging"
                else:
                    self.tasks_completed += 1
                    self.destination = None
                    self.path = []
                    self.status = "idle"
            return self.position

        next_pos = self.path[0]

        # Blocked by robot
        if next_pos in occupied_next:
            self.status = "blocked"
            return self.position

        # Blocked by obstacle
        if not floor.is_walkable(next_pos[0], next_pos[1]):
            self.status = "recalculating"
            self.recalculate(floor, algorithm, congestion_map)
            return self.position

        # Move forward
        self.trail_history.append(self.position)
        if len(self.trail_history) > TRAIL_MAX_LENGTH:
            self.trail_history.pop(0)
        self.position = next_pos
        self.path.pop(0)
        self.total_steps += 1
        self.status = "moving"

        # Battery drain
        cell_type = floor.grid[self.position[0], self.position[1]]
        drain = BATTERY_DRAIN_PER_STEP
        if cell_type == SLOW_ZONE:
            drain += BATTERY_DRAIN_SLOW_ZONE
        self.battery = max(0, self.battery - drain)

        # Arrived?
        if not self.path:
            if self.destination and self.position == self.destination:
                if self._returning_to_charge:
                    self.is_charging = True
                    self.status = "charging"
                else:
                    self.tasks_completed += 1
                    self.destination = None
                    self.status = "idle"

        return self.position


class FleetManager:
    """Coordinates AGVs with congestion-aware dispatching and battery management."""

    def __init__(self, floor, algorithm: Algorithm = Algorithm.ASTAR):
        self.floor = floor
        self.algorithm = algorithm
        self.robots: list[AGV] = []
        self._dispatch_timer = 0
        self.task_queue: list[tuple] = []   # pending (source, dest) tasks
        self._congestion_map: dict = {}

    # -- fleet setup ------------------------------------------------------
    def spawn_initial(self, count: int = INITIAL_ROBOT_COUNT):
        load_stations = self.floor.get_stations(STATION_LOAD)
        for i in range(min(count, len(load_stations))):
            agv = AGV(position=load_stations[i])
            self.robots.append(agv)

    def add_robot(self, position: tuple):
        if self.floor.is_walkable(position[0], position[1]):
            agv = AGV(position=position)
            self.robots.append(agv)
            return agv
        return None

    # -- congestion map ---------------------------------------------------
    def _build_congestion_map(self):
        """Count how many robots are within CONGESTION_RADIUS of each cell."""
        cmap: dict[tuple, int] = {}
        for robot in self.robots:
            if robot.status in ("idle", "charging"):
                continue
            r, c = robot.position
            for dr in range(-CONGESTION_RADIUS, CONGESTION_RADIUS + 1):
                for dc in range(-CONGESTION_RADIUS, CONGESTION_RADIUS + 1):
                    key = (r + dr, c + dc)
                    cmap[key] = cmap.get(key, 0) + 1
        self._congestion_map = cmap

    # -- task queue -------------------------------------------------------
    def enqueue_tasks(self, count: int = 3):
        """Generate random pending tasks for the queue."""
        deliver = self.floor.get_stations(STATION_DELIVER)
        load = self.floor.get_stations(STATION_LOAD)
        if not deliver or not load:
            return
        for _ in range(count):
            src = random.choice(load)
            dst = random.choice(deliver)
            self.task_queue.append((src, dst))

    # -- dispatching ------------------------------------------------------
    def auto_dispatch(self):
        """Assign idle robots to tasks from the queue or random stations."""
        deliver_stations = self.floor.get_stations(STATION_DELIVER)
        load_stations = self.floor.get_stations(STATION_LOAD)
        if not deliver_stations or not load_stations:
            return

        for robot in self.robots:
            if robot.status != "idle" or robot.destination is not None:
                continue
            if robot.is_charging or robot._returning_to_charge:
                continue

            # Low battery? Go charge instead
            if robot.battery < BATTERY_LOW_THRESHOLD:
                robot.seek_charger(self.floor, self.algorithm,
                                   self._congestion_map)
                continue

            # Try task queue first
            if self.task_queue:
                src, dst = self.task_queue.pop(0)
                robot.assign_task(dst, self.floor, self.algorithm,
                                  self._congestion_map)
                continue

            # Fallback: alternate between load and deliver
            at_load = robot.position in load_stations
            targets = deliver_stations if at_load else load_stations
            dest = random.choice(targets)
            robot.assign_task(dest, self.floor, self.algorithm,
                              self._congestion_map)

    # -- step all ---------------------------------------------------------
    def step_all(self):
        """Advance every robot one tick."""
        self._dispatch_timer += 1
        if self._dispatch_timer >= AUTO_DISPATCH_INTERVAL:
            self._dispatch_timer = 0
            # Refill task queue if running low
            if len(self.task_queue) < 5:
                self.enqueue_tasks(3)
            self._build_congestion_map()
            self.auto_dispatch()

        occupied_next: set[tuple] = set()
        for robot in sorted(self.robots, key=lambda r: r.id):
            new_pos = robot.step(self.floor, occupied_next, self.algorithm,
                                 self._congestion_map)
            occupied_next.add(new_pos)

    # -- queries ----------------------------------------------------------
    def get_all_paths(self) -> dict[int, list[tuple]]:
        return {r.id: r.path for r in self.robots}
