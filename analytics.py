"""
analytics.py -- Live simulation statistics and heatmap tracker.
"""

import numpy as np
from config import GRID_ROWS, GRID_COLS, HEATMAP_COLORS


class SimulationAnalytics:
    """Collects and exposes live metrics for the UI overlay."""

    def __init__(self):
        self.tick = 0
        self.total_tasks_completed = 0
        self.total_recalculations = 0
        self.total_steps = 0
        self.collisions_avoided = 0
        self._prev_tasks = {}      # robot_id -> tasks_completed snapshot
        self._prev_recalcs = {}    # robot_id -> recalculations snapshot
        # heatmap: counts how many times each cell has been visited
        self.heatmap = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    def update(self, robots):
        """Call once per simulation tick with the fleet's robot list."""
        self.tick += 1
        for r in robots:
            # record cell visit for heatmap
            self.heatmap[r.position[0], r.position[1]] += 1

            # detect new task completions
            prev_t = self._prev_tasks.get(r.id, 0)
            if r.tasks_completed > prev_t:
                self.total_tasks_completed += (r.tasks_completed - prev_t)
            self._prev_tasks[r.id] = r.tasks_completed

            # detect new recalculations
            prev_r = self._prev_recalcs.get(r.id, 0)
            if r.recalculations > prev_r:
                self.total_recalculations += (r.recalculations - prev_r)
            self._prev_recalcs[r.id] = r.recalculations

            # steps
            self.total_steps = sum(bot.total_steps for bot in robots)

            # collisions avoided = number of robots currently blocked
            if r.status == "blocked":
                self.collisions_avoided += 1

    def get_heatmap_color(self, r: int, c: int):
        """Return an RGB color for the heatmap intensity at (r, c)."""
        count = self.heatmap[r, c]
        if count == 0:
            return None   # no overlay
        max_count = max(self.heatmap.max(), 1)
        ratio = min(count / max_count, 1.0)
        idx = min(int(ratio * (len(HEATMAP_COLORS) - 1)), len(HEATMAP_COLORS) - 1)
        return HEATMAP_COLORS[idx]

    def get_summary(self, robots) -> dict:
        """Return a dictionary of display-ready stats."""
        active = sum(1 for r in robots if r.status == "moving")
        idle = sum(1 for r in robots if r.status == "idle")
        blocked = sum(1 for r in robots if r.status in ("blocked", "recalculating"))
        avg_path = 0
        paths = [len(r.path) for r in robots if r.path]
        if paths:
            avg_path = sum(paths) / len(paths)
        return {
            "tick": self.tick,
            "robots_total": len(robots),
            "robots_active": active,
            "robots_idle": idle,
            "robots_blocked": blocked,
            "tasks_completed": self.total_tasks_completed,
            "recalculations": self.total_recalculations,
            "total_steps": self.total_steps,
            "avg_remaining_path": round(avg_path, 1),
            "collisions_avoided": self.collisions_avoided,
        }
