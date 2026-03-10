"""
analytics.py -- Live simulation statistics, heatmap, and per-robot efficiency export.
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
        self.deadlocks_resolved = 0
        self._prev_tasks = {}
        self._prev_recalcs = {}
        self._prev_deadlocks = {}
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

            prev_d = self._prev_deadlocks.get(r.id, 0)
            if r.deadlocks_resolved > prev_d:
                self.deadlocks_resolved += (r.deadlocks_resolved - prev_d)
            self._prev_deadlocks[r.id] = r.deadlocks_resolved

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
        avg_eff = 0
        if robots:
            avg_battery = sum(r.battery for r in robots) / len(robots)
            avg_eff = sum(r.efficiency for r in robots) / len(robots)
        return {
            "tick": self.tick,
            "robots_total": len(robots),
            "robots_active": active,
            "robots_idle": idle,
            "robots_blocked": blocked,
            "robots_charging": charging,
            "tasks_completed": self.total_tasks_completed,
            "recalculations": self.total_recalculations,
            "deadlocks": self.deadlocks_resolved,
            "total_steps": self.total_steps,
            "avg_remaining_path": round(avg_path, 1),
            "avg_battery": round(avg_battery, 1),
            "avg_efficiency": round(avg_eff * 100, 1),
            "collisions_avoided": self.collisions_avoided,
        }

    def export_heatmap_csv(self):
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

    def export_robot_stats_csv(self, robots):
        """Export per-robot efficiency stats."""
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        path = os.path.join(SCREENSHOT_DIR, f"robot_stats_tick{self.tick}.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "robot_id", "tasks_completed", "total_steps",
                "ticks_moving", "ticks_blocked", "ticks_idle",
                "ticks_charging", "efficiency_pct",
                "recalculations", "deadlocks_resolved",
                "battery", "avg_path_length",
            ])
            for r in robots:
                avg_pl = 0
                if r.total_path_lengths:
                    avg_pl = round(sum(r.total_path_lengths) /
                                   len(r.total_path_lengths), 1)
                writer.writerow([
                    r.id, r.tasks_completed, r.total_steps,
                    r.ticks_moving, r.ticks_blocked, r.ticks_idle,
                    r.ticks_charging, round(r.efficiency * 100, 1),
                    r.recalculations, r.deadlocks_resolved,
                    round(r.battery, 1), avg_pl,
                ])
        return path
