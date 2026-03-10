"""
environment.py -- Factory floor grid with obstacles, stations, and layout generation.
"""

import numpy as np
from config import (
    GRID_ROWS, GRID_COLS, EMPTY, WALL,
    STATION_LOAD, STATION_DELIVER, DYNAMIC_OBSTACLE,
    SLOW_ZONE, CHARGE_STATION, CELL_COST,
)


class FactoryFloor:
    """2-D grid representing the factory floor."""

    def __init__(self, rows: int = GRID_ROWS, cols: int = GRID_COLS,
                 layout_id: int = 1):
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=int)
        self.layout_id = layout_id
        self.set_layout(layout_id)

    # -- queries ----------------------------------------------------------
    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_walkable(self, r: int, c: int) -> bool:
        if not self.in_bounds(r, c):
            return False
        return self.grid[r, c] not in (WALL, DYNAMIC_OBSTACLE)

    def cell_cost(self, r: int, c: int) -> float:
        """Movement cost for entering cell (r, c)."""
        return CELL_COST.get(self.grid[r, c], 1)

    def get_neighbors(self, r: int, c: int):
        """Return walkable 4-directional neighbors."""
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = r + dr, c + dc
            if self.is_walkable(nr, nc):
                yield (nr, nc)

    def get_stations(self, station_type: int):
        """Return list of (r, c) for a given station type."""
        coords = np.argwhere(self.grid == station_type)
        return [tuple(rc) for rc in coords]

    # -- mutations --------------------------------------------------------
    def toggle_wall(self, r: int, c: int):
        """Toggle a cell between EMPTY and DYNAMIC_OBSTACLE."""
        if not self.in_bounds(r, c):
            return
        cell = self.grid[r, c]
        if cell == EMPTY:
            self.grid[r, c] = DYNAMIC_OBSTACLE
        elif cell == DYNAMIC_OBSTACLE:
            self.grid[r, c] = EMPTY

    def place_obstacle(self, r: int, c: int):
        if self.in_bounds(r, c) and self.grid[r, c] == EMPTY:
            self.grid[r, c] = DYNAMIC_OBSTACLE

    def remove_obstacle(self, r: int, c: int):
        if self.in_bounds(r, c) and self.grid[r, c] == DYNAMIC_OBSTACLE:
            self.grid[r, c] = EMPTY

    # -- layout management ------------------------------------------------
    def set_layout(self, layout_id: int):
        """Switch to a different factory layout (1, 2, or 3)."""
        self.grid[:] = EMPTY
        self.layout_id = layout_id
        if layout_id == 1:
            self._layout_factory()
        elif layout_id == 2:
            self._layout_warehouse()
        elif layout_id == 3:
            self._layout_open_plan()
        else:
            self._layout_factory()

    # -- helpers ----------------------------------------------------------
    def _borders(self):
        g = self.grid
        g[0, :] = WALL; g[-1, :] = WALL
        g[:, 0] = WALL; g[:, -1] = WALL

    def _ensure_reachable(self, rows, cols_list):
        g = self.grid
        for r in rows:
            for c in cols_list:
                if g[r, c] == WALL:
                    g[r, c] = EMPTY

    # -- Layout 1: Factory ------------------------------------------------
    def _layout_factory(self):
        g = self.grid
        self._borders()

        # Machine clusters
        for r in range(3, 6):
            for c in range(3, 7):    g[r, c] = WALL
        for r in range(3, 6):
            for c in range(10, 14):  g[r, c] = WALL
        for r in range(9, 12):
            for c in range(6, 10):   g[r, c] = WALL
        for r in range(14, 17):
            for c in range(3, 7):    g[r, c] = WALL
        for r in range(9, 12):
            for c in range(16, 20):  g[r, c] = WALL
        for r in range(14, 17):
            for c in range(22, 27):  g[r, c] = WALL

        # Conveyor belts
        for c in range(14, 22): g[4, c] = WALL
        g[4, 17] = EMPTY
        for c in range(10, 16): g[15, c] = WALL
        g[15, 12] = EMPTY

        # Vertical partitions
        for r in range(6, 9):   g[r, 14] = WALL
        for r in range(12, 15): g[r, 23] = WALL

        # Pillars
        for pos in [(7,4),(7,10),(13,4),(13,18),(7,22),(17,10)]:
            g[pos] = WALL

        # Slow zones (near heavy machinery)
        for r in range(6, 9):
            for c in range(3, 7):
                if g[r, c] == EMPTY: g[r, c] = SLOW_ZONE
        for r in range(12, 14):
            for c in range(16, 20):
                if g[r, c] == EMPTY: g[r, c] = SLOW_ZONE

        # Charging stations (mid-left and mid-right)
        g[9, 2] = CHARGE_STATION
        g[9, GRID_COLS - 3] = CHARGE_STATION

        # Load and deliver stations
        for r in [2, 6, 10, 14, 17]:
            g[r, 1] = STATION_LOAD
            g[r, GRID_COLS - 2] = STATION_DELIVER
        self._ensure_reachable([2, 6, 10, 14, 17], [2, GRID_COLS - 3])

    # -- Layout 2: Warehouse ----------------------------------------------
    def _layout_warehouse(self):
        g = self.grid
        self._borders()

        for row_start in [3, 7, 11, 15]:
            for c in range(3, GRID_COLS - 3):
                if c % 8 < 5:
                    g[row_start, c] = WALL
                    g[row_start + 1, c] = WALL

        mid_c = GRID_COLS // 2
        for r in range(1, self.rows - 1):
            g[r, mid_c] = EMPTY
            g[r, mid_c - 1] = EMPTY

        # Slow zones in narrow cross-aisles
        for row_start in [3, 7, 11, 15]:
            for c in range(3, GRID_COLS - 3):
                if c % 8 >= 5:  # gap columns
                    for rr in [row_start, row_start + 1]:
                        if g[rr, c] == EMPTY:
                            g[rr, c] = SLOW_ZONE

        # Charging stations
        g[5, mid_c] = CHARGE_STATION
        g[13, mid_c] = CHARGE_STATION

        for c in [3, 8, 14, 20, 25]:
            if c < GRID_COLS - 1:
                g[1, c] = STATION_LOAD
                g[self.rows - 2, c] = STATION_DELIVER
                if g[2, c] == WALL: g[2, c] = EMPTY
                if g[self.rows - 3, c] == WALL: g[self.rows - 3, c] = EMPTY

    # -- Layout 3: Open Plan ----------------------------------------------
    def _layout_open_plan(self):
        g = self.grid
        self._borders()

        pillars = [
            (4, 5), (4, 12), (4, 20), (4, 25),
            (9, 8), (9, 15), (9, 22),
            (14, 5), (14, 12), (14, 20), (14, 25),
        ]
        for (pr, pc) in pillars:
            for dr in range(2):
                for dc in range(2):
                    if self.in_bounds(pr + dr, pc + dc):
                        g[pr + dr, pc + dc] = WALL

        for r in range(2, 5):
            for c in range(GRID_COLS - 6, GRID_COLS - 3):
                g[r, c] = WALL
        for c in range(GRID_COLS - 6, GRID_COLS - 2):
            g[2, c] = WALL

        for r in range(self.rows - 5, self.rows - 2):
            for c in range(3, 6):
                g[r, c] = WALL
        for c in range(3, 8):
            g[self.rows - 3, c] = WALL

        # Slow zones around pillar clusters
        for (pr, pc) in pillars:
            for dr in range(-1, 3):
                for dc in range(-1, 3):
                    rr, cc = pr + dr, pc + dc
                    if self.in_bounds(rr, cc) and g[rr, cc] == EMPTY:
                        g[rr, cc] = SLOW_ZONE

        # Charging stations
        g[10, 3] = CHARGE_STATION
        g[10, GRID_COLS - 4] = CHARGE_STATION

        for r in [2, 6, 10, 14, 17]:
            g[r, 1] = STATION_LOAD
            g[r, GRID_COLS - 2] = STATION_DELIVER
        self._ensure_reachable([2, 6, 10, 14, 17], [2, GRID_COLS - 3])
