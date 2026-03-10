"""
robot.py -- AGV with battery, deadlock detection, per-robot efficiency stats,
and FleetManager with congestion-aware routing and task queue.
"""

import random
from config import (
    Algorithm, ROBOT_COLORS, INITIAL_ROBOT_COUNT, AUTO_DISPATCH_INTERVAL,
    STATION_LOAD, STATION_DELIVER, CHARGE_STATION, SLOW_ZONE,
    TRAIL_MAX_LENGTH, CONGESTION_RADIUS,
    BATTERY_MAX, BATTERY_DRAIN_PER_STEP, BATTERY_DRAIN_SLOW_ZONE,
    BATTERY_CHARGE_RATE, BATTERY_LOW_THRESHOLD,
    DEADLOCK_TIMEOUT, DEADLOCK_JITTER,
)
from pathfinding import find_path


class AGV:
    """A single autonomous guided vehicle with battery and efficiency tracking."""

    _id_counter = 0

    def __init__(self, position: tuple, color: tuple | None = None):
        AGV._id_counter += 1
        self.id = AGV._id_counter
        self.position = position
        self.destination = None
        self.path: list[tuple] = []
        self.status = "idle"
        self.color = color or ROBOT_COLORS[(self.id - 1) % len(ROBOT_COLORS)]
        self.tasks_completed = 0
        self.total_steps = 0
        self.recalculations = 0
        self.trail_history: list[tuple] = []
        # Battery
        self.battery = BATTERY_MAX
        self.is_charging = False
        self._returning_to_charge = False
        self.pending_delivery = None
        # Deadlock detection
        self.blocked_ticks = 0
        self.deadlocks_resolved = 0
        # Efficiency stats
        self.ticks_moving = 0
        self.ticks_blocked = 0
        self.ticks_idle = 0
        self.ticks_charging = 0
        self.total_path_lengths: list[int] = []  # length of each assigned path

    @property
    def efficiency(self) -> float:
        """Fraction of active time spent moving (0.0 to 1.0)."""
        total = self.ticks_moving + self.ticks_blocked
        return self.ticks_moving / total if total > 0 else 0.0

    # -- task management --------------------------------------------------
    def assign_task(self, destination, floor, algorithm, congestion_map=None):
        self.destination = destination
        self._returning_to_charge = False
        self.blocked_ticks = 0
        self.path = find_path(floor, self.position, destination,
                              algorithm, congestion_map)
        if self.path:
            self.total_path_lengths.append(len(self.path))
            self.path.pop(0)
            self.status = "moving"
        else:
            self.status = "blocked"

    def recalculate(self, floor, algorithm, congestion_map=None,
                    jitter: float = 0.0):
        self.recalculations += 1
        self.blocked_ticks = 0
        if self.destination is None:
            self.status = "idle"
            return
        self.path = find_path(floor, self.position, self.destination,
                              algorithm, congestion_map, jitter=jitter)
        if self.path:
            self.path.pop(0)
            self.status = "moving"
        else:
            self.status = "blocked"

    def seek_charger(self, floor, algorithm, congestion_map=None):
        chargers = floor.get_stations(CHARGE_STATION)
        if not chargers:
            return
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
        # Tick efficiency counters
        if self.status == "moving":
            self.ticks_moving += 1
        elif self.status in ("blocked", "recalculating"):
            self.ticks_blocked += 1
        elif self.status == "idle":
            self.ticks_idle += 1
        elif self.status == "charging":
            self.ticks_charging += 1

        # Charging
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

        # Blocked by robot or moving obstacle
        if next_pos in occupied_next:
            self.status = "blocked"
            self.blocked_ticks += 1
            # Deadlock detection
            if self.blocked_ticks >= DEADLOCK_TIMEOUT:
                self.deadlocks_resolved += 1
                self.recalculate(floor, algorithm, congestion_map,
                                 jitter=DEADLOCK_JITTER)
            return self.position

        # Blocked by obstacle
        if not floor.is_walkable(next_pos[0], next_pos[1]):
            self.status = "recalculating"
            self.recalculate(floor, algorithm, congestion_map)
            return self.position

        # Move forward
        self.blocked_ticks = 0
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
                elif self.pending_delivery:
                    next_dest = self.pending_delivery
                    self.pending_delivery = None
                    self.assign_task(next_dest, floor, algorithm, congestion_map)
                else:
                    self.tasks_completed += 1
                    self.destination = None
                    self.status = "idle"

        return self.position


class FleetManager:
    """Coordinates AGVs with congestion-aware dispatching, deadlock recovery,
    and task queue management."""

    def __init__(self, floor, algorithm: Algorithm = Algorithm.ASTAR):
        self.floor = floor
        self.algorithm = algorithm
        self.robots: list[AGV] = []
        self._dispatch_timer = 0
        self.task_queue: list[tuple] = []
        self._congestion_map: dict = {}

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

    def _build_congestion_map(self):
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

    def enqueue_tasks(self, count: int = 3):
        deliver = self.floor.get_stations(STATION_DELIVER)
        load = self.floor.get_stations(STATION_LOAD)
        if not deliver or not load:
            return
        for _ in range(count):
            src = random.choice(load)
            dst = random.choice(deliver)
            self.task_queue.append((src, dst))

    def auto_dispatch(self):
        deliver_stations = self.floor.get_stations(STATION_DELIVER)
        load_stations = self.floor.get_stations(STATION_LOAD)
        if not deliver_stations or not load_stations:
            return

        claimed = {r.destination for r in self.robots if r.destination is not None}

        for robot in self.robots:
            if robot.status != "idle" or robot.destination is not None:
                continue
            if robot.is_charging or robot._returning_to_charge:
                continue

            if robot.battery < BATTERY_LOW_THRESHOLD:
                robot.seek_charger(self.floor, self.algorithm,
                                   self._congestion_map)
                continue

            if self.task_queue:
                src, dst = self.task_queue.pop(0)
                robot.pending_delivery = dst
                robot.assign_task(src, self.floor, self.algorithm,
                                  self._congestion_map)
                claimed.add(dst)
                continue

            at_load = robot.position in load_stations
            targets = deliver_stations if at_load else load_stations

            available = [t for t in targets if t not in claimed]
            if available:
                dest = random.choice(available)
                robot.assign_task(dest, self.floor, self.algorithm,
                                  self._congestion_map)
                claimed.add(dest)

    def step_all(self, moving_obs_positions: set | None = None):
        """Advance every robot one tick."""
        self._build_congestion_map()

        self._dispatch_timer += 1
        if self._dispatch_timer >= AUTO_DISPATCH_INTERVAL:
            self._dispatch_timer = 0
            if len(self.task_queue) < 5:
                self.enqueue_tasks(3)
            self.auto_dispatch()

        occupied: set[tuple] = {r.position for r in self.robots}
        if moving_obs_positions:
            occupied |= moving_obs_positions

        for robot in sorted(self.robots, key=lambda r: r.id):
            occupied.discard(robot.position)
            new_pos = robot.step(self.floor, occupied, self.algorithm,
                                 self._congestion_map)
            occupied.add(new_pos)

    def get_all_paths(self) -> dict[int, list[tuple]]:
        return {r.id: r.path for r in self.robots}
