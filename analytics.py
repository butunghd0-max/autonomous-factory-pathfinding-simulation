"""
analytics.py -- Live simulation statistics, heatmap, and CSV export.
"""

import os
import csv
import numpy as np
from config import GRID_ROWS, GRID_COLS, HEATMAP_COLORS, SCREENSHOT_DIR


class SimulationAnalytics:
    """Collects live metrics, heatmap data, and supports CSV export."""

    def __init__(self):
        self.tick = 0
        self.total_tasks_completed = 0
        self.total_recalculations = 0
        self.total_steps = 0
        self.collisions_avoided = 0
        self._prev_tasks = {}
        self._prev_recalcs = {}
        self.heatmap = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)

    def update(self, robots):
        self.tick += 1
        for r in robots:
            self.heatmap[r.position[0], r.position[1]] += 1

            prev_t = self._prev_tasks.get(r.id, 0)
            if r.tasks_completed > prev_t:
                self.total_tasks_completed += (r.tasks_completed - prev_t)
            self._prev_tasks[r.id] = r.tasks_completed

            prev_r = self._prev_recalcs.get(r.id, 0)
            if r.recalculations > prev_r:
                self.total_recalculations += (r.recalculations - prev_r)
            self._prev_recalcs[r.id] = r.recalculations

            self.total_steps = sum(bot.total_steps for bot in robots)

            if r.status == "blocked":
                self.collisions_avoided += 1

    def get_heatmap_color(self, r: int, c: int):
        count = self.heatmap[r, c]
        if count == 0:
            return None
        max_count = max(self.heatmap.max(), 1)
        ratio = min(count / max_count, 1.0)
        idx = min(int(ratio * (len(HEATMAP_COLORS) - 1)),
                  len(HEATMAP_COLORS) - 1)
        return HEATMAP_COLORS[idx]

    def get_summary(self, robots) -> dict:
        active = sum(1 for r in robots if r.status == "moving")
        idle = sum(1 for r in robots if r.status == "idle")
        blocked = sum(1 for r in robots
                      if r.status in ("blocked", "recalculating"))
        charging = sum(1 for r in robots if r.status == "charging")
        avg_path = 0
        paths = [len(r.path) for r in robots if r.path]
        if paths:
            avg_path = sum(paths) / len(paths)
        avg_battery = 0
        if robots:
            avg_battery = sum(r.battery for r in robots) / len(robots)
        return {
            "tick": self.tick,
            "robots_total": len(robots),
            "robots_active": active,
            "robots_idle": idle,
            "robots_blocked": blocked,
            "robots_charging": charging,
            "tasks_completed": self.total_tasks_completed,
            "recalculations": self.total_recalculations,
            "total_steps": self.total_steps,
            "avg_remaining_path": round(avg_path, 1),
            "avg_battery": round(avg_battery, 1),
            "collisions_avoided": self.collisions_avoided,
        }

    # -- export heatmap to CSV --------------------------------------------
    def export_heatmap_csv(self):
        """Save heatmap data as a CSV file. Returns the file path."""
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        path = os.path.join(SCREENSHOT_DIR, f"heatmap_tick{self.tick}.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["row", "col", "visit_count"])
            for r in range(GRID_ROWS):
                for c in range(GRID_COLS):
                    if self.heatmap[r, c] > 0:
                        writer.writerow([r, c, int(self.heatmap[r, c])])
        return path
