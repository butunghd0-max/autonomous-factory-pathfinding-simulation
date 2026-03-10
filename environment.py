"""
environment.py — Factory floor grid with obstacles, stations, and layout generation.
"""

import numpy as np
from config import (
    GRID_ROWS, GRID_COLS, EMPTY, WALL,
    STATION_LOAD, STATION_DELIVER, DYNAMIC_OBSTACLE,
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

    # ── queries ──────────────────────────────────────────────────────────
    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_walkable(self, r: int, c: int) -> bool:
        if not self.in_bounds(r, c):
            return False
        return self.grid[r, c] != WALL and self.grid[r, c] != DYNAMIC_OBSTACLE

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

    # ── mutations ────────────────────────────────────────────────────────
    def toggle_wall(self, r: int, c: int):
        """Toggle a cell between EMPTY and DYNAMIC_OBSTACLE.
        Does nothing on station cells."""
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

    # ── layout management ────────────────────────────────────────────────
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

    # ── Layout 1: Factory (original) ─────────────────────────────────────
    def _layout_factory(self):
        """Machine clusters, conveyor belts, partitions."""
        g = self.grid
        g[0, :] = WALL; g[-1, :] = WALL
        g[:, 0] = WALL; g[:, -1] = WALL

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

        # Stations
        for r in [2, 6, 10, 14, 17]:
            g[r, 1] = STATION_LOAD
            g[r, GRID_COLS - 2] = STATION_DELIVER
        for r in [2, 6, 10, 14, 17]:
            for c in [2, GRID_COLS - 3]:
                if g[r, c] == WALL: g[r, c] = EMPTY

    # ── Layout 2: Warehouse (long aisles with shelving) ──────────────────
    def _layout_warehouse(self):
        """Parallel shelving rows with cross-aisles -- warehouse style."""
        g = self.grid
        g[0, :] = WALL; g[-1, :] = WALL
        g[:, 0] = WALL; g[:, -1] = WALL

        # Horizontal shelf rows with gaps every 6 columns
        for row_start in [3, 7, 11, 15]:
            for c in range(3, GRID_COLS - 3):
                if c % 8 < 5:      # shelf block of 5, gap of 3
                    g[row_start, c] = WALL
                    g[row_start + 1, c] = WALL

        # Central vertical aisle (always clear)
        mid_c = GRID_COLS // 2
        for r in range(1, self.rows - 1):
            g[r, mid_c] = EMPTY
            g[r, mid_c - 1] = EMPTY

        # Stations along top and bottom
        for c in [3, 8, 14, 20, 25]:
            if c < GRID_COLS - 1:
                g[1, c] = STATION_LOAD
                g[self.rows - 2, c] = STATION_DELIVER
                # ensure reachable
                if g[2, c] == WALL: g[2, c] = EMPTY
                if g[self.rows - 3, c] == WALL: g[self.rows - 3, c] = EMPTY

    # ── Layout 3: Open plan (scattered pillars, wide aisles) ─────────────
    def _layout_open_plan(self):
        """Minimal obstacles -- open floor with scattered pillars and zones."""
        g = self.grid
        g[0, :] = WALL; g[-1, :] = WALL
        g[:, 0] = WALL; g[:, -1] = WALL

        # Scattered 2x2 pillar blocks
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

        # L-shaped assembly zone (top-right)
        for r in range(2, 5):
            for c in range(GRID_COLS - 6, GRID_COLS - 3):
                g[r, c] = WALL
        for c in range(GRID_COLS - 6, GRID_COLS - 2):
            g[2, c] = WALL

        # L-shaped assembly zone (bottom-left)
        for r in range(self.rows - 5, self.rows - 2):
            for c in range(3, 6):
                g[r, c] = WALL
        for c in range(3, 8):
            g[self.rows - 3, c] = WALL

        # Stations (left = load, right = deliver)
        for r in [2, 6, 10, 14, 17]:
            g[r, 1] = STATION_LOAD
            g[r, GRID_COLS - 2] = STATION_DELIVER
        for r in [2, 6, 10, 14, 17]:
            for c in [2, GRID_COLS - 3]:
                if g[r, c] == WALL: g[r, c] = EMPTY
