"""
renderer.py -- Pygame rendering with battery bars, task queue, slow zones,
charge stations, screenshot export, and all overlay modes.
"""

import os
import pygame
from config import (
    CELL_SIZE, GRID_ROWS, GRID_COLS,
    WINDOW_WIDTH, WINDOW_HEIGHT, PANEL_WIDTH, FPS,
    BG_COLOR, GRID_LINE_COLOR, EMPTY_COLOR, WALL_COLOR,
    STATION_LOAD_CLR, STATION_DELIVER_CLR, DYNAMIC_OBS_COLOR,
    SLOW_ZONE_COLOR, CHARGE_STATION_CLR,
    PANEL_BG, PANEL_TEXT_COLOR, PANEL_ACCENT, PANEL_HEADER_COLOR,
    PATH_ALPHA, TRAIL_DECAY,
    EMPTY, WALL, STATION_LOAD, STATION_DELIVER, DYNAMIC_OBSTACLE,
    SLOW_ZONE, CHARGE_STATION,
    BASE_MOVE_INTERVAL, MIN_MOVE_INTERVAL, MAX_MOVE_INTERVAL,
    Algorithm, LAYOUT_COUNT, BATTERY_MAX, SCREENSHOT_DIR,
)

_CELL_COLORS = {
    EMPTY:            EMPTY_COLOR,
    WALL:             WALL_COLOR,
    STATION_LOAD:     STATION_LOAD_CLR,
    STATION_DELIVER:  STATION_DELIVER_CLR,
    DYNAMIC_OBSTACLE: DYNAMIC_OBS_COLOR,
    SLOW_ZONE:        SLOW_ZONE_COLOR,
    CHARGE_STATION:   CHARGE_STATION_CLR,
}

_CELL_LABELS = {
    STATION_LOAD: ("L", (255, 255, 255)),
    STATION_DELIVER: ("D", (255, 255, 255)),
    CHARGE_STATION: ("C", (20, 20, 30)),
    SLOW_ZONE: ("~", (160, 140, 200)),
}


class Renderer:
    """Manages the pygame window, drawing, and user-input events."""

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Autonomous Factory Pathfinding Simulation")
        self.clock = pygame.time.Clock()
        self.font_sm = pygame.font.SysFont("consolas", 13)
        self.font_md = pygame.font.SysFont("consolas", 15, bold=True)
        self.font_lg = pygame.font.SysFont("consolas", 20, bold=True)
        self.font_xs = pygame.font.SysFont("consolas", 10)
        self.path_surface = pygame.Surface(
            (GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE), pygame.SRCALPHA)
        self.show_analytics = True
        self.show_heatmap = False
        self.show_trails = True
        self._screenshot_flash = 0
        self._export_flash = 0

    # -- main draw --------------------------------------------------------
    def draw(self, floor, fleet, analytics_summary, algorithm,
             move_interval, paused, analytics_obj=None):
        self.screen.fill(BG_COLOR)
        self._draw_grid(floor)
        if self.show_heatmap and analytics_obj:
            self._draw_heatmap(analytics_obj)
        if self.show_trails:
            self._draw_trails(fleet)
        self._draw_paths(fleet)
        self._draw_robots(fleet)
        self._draw_panel(analytics_summary, algorithm, move_interval,
                         paused, fleet, floor.layout_id)
        # Flash effects for screenshot/export
        if self._screenshot_flash > 0:
            flash = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT))
            flash.fill((255, 255, 255))
            flash.set_alpha(self._screenshot_flash)
            self.screen.blit(flash, (0, 0))
            self._screenshot_flash -= 15
        if self._export_flash > 0:
            txt = self.font_md.render("Heatmap exported!", True, (46, 204, 113))
            self.screen.blit(txt, (10, WINDOW_HEIGHT - 30))
            self._export_flash -= 1
        pygame.display.flip()
        self.clock.tick(FPS)

    # -- grid -------------------------------------------------------------
    def _draw_grid(self, floor):
        for r in range(floor.rows):
            for c in range(floor.cols):
                cell = floor.grid[r, c]
                color = _CELL_COLORS.get(cell, EMPTY_COLOR)
                rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE,
                                   CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, GRID_LINE_COLOR, rect, 1)

                # Cell labels
                if cell in _CELL_LABELS:
                    label, lcolor = _CELL_LABELS[cell]
                    txt = self.font_sm.render(label, True, lcolor)
                    self.screen.blit(
                        txt, (c * CELL_SIZE + CELL_SIZE // 3,
                              r * CELL_SIZE + CELL_SIZE // 4))

    # -- heatmap overlay --------------------------------------------------
    def _draw_heatmap(self, analytics_obj):
        heat_surf = pygame.Surface(
            (GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE), pygame.SRCALPHA)
        for r in range(GRID_ROWS):
            for c in range(GRID_COLS):
                color = analytics_obj.get_heatmap_color(r, c)
                if color is None:
                    continue
                rect = pygame.Rect(c * CELL_SIZE, r * CELL_SIZE,
                                   CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(heat_surf, (*color, 120), rect)
        self.screen.blit(heat_surf, (0, 0))

    # -- robot trail history (fading) -------------------------------------
    def _draw_trails(self, fleet):
        trail_surf = pygame.Surface(
            (GRID_COLS * CELL_SIZE, GRID_ROWS * CELL_SIZE), pygame.SRCALPHA)
        for robot in fleet.robots:
            if not robot.trail_history:
                continue
            n = len(robot.trail_history)
            for i, (r, c) in enumerate(robot.trail_history):
                age_ratio = (i + 1) / n
                alpha = int(50 * age_ratio * TRAIL_DECAY)
                color = (*robot.color, max(alpha, 10))
                size = max(int((CELL_SIZE - 12) * age_ratio), 4)
                cx = c * CELL_SIZE + CELL_SIZE // 2
                cy = r * CELL_SIZE + CELL_SIZE // 2
                pygame.draw.circle(trail_surf, color, (cx, cy), size // 2)
        self.screen.blit(trail_surf, (0, 0))

    # -- robot path trails (future) ----------------------------------------
    def _draw_paths(self, fleet):
        self.path_surface.fill((0, 0, 0, 0))
        for robot in fleet.robots:
            if not robot.path:
                continue
            color = (*robot.color, PATH_ALPHA)
            for (r, c) in robot.path:
                rect = pygame.Rect(c * CELL_SIZE + 4, r * CELL_SIZE + 4,
                                   CELL_SIZE - 8, CELL_SIZE - 8)
                pygame.draw.rect(self.path_surface, color, rect,
                                 border_radius=4)
        self.screen.blit(self.path_surface, (0, 0))

    # -- robots with battery bars -----------------------------------------
    def _draw_robots(self, fleet):
        for robot in fleet.robots:
            r, c = robot.position
            cx = c * CELL_SIZE + CELL_SIZE // 2
            cy = r * CELL_SIZE + CELL_SIZE // 2
            radius = CELL_SIZE // 2 - 3

            # Glow
            glow_color = (*robot.color, 60)
            glow_surf = pygame.Surface((CELL_SIZE, CELL_SIZE), pygame.SRCALPHA)
            pygame.draw.circle(glow_surf, glow_color,
                               (CELL_SIZE // 2, CELL_SIZE // 2), radius + 4)
            self.screen.blit(glow_surf, (c * CELL_SIZE, r * CELL_SIZE))

            # Body
            body_color = robot.color
            if robot.status == "charging":
                # pulse effect
                pulse = abs((self.clock.get_rawtime() % 1000) - 500) / 500
                body_color = tuple(min(255, int(ch + 60 * pulse))
                                   for ch in robot.color)
            pygame.draw.circle(self.screen, body_color, (cx, cy), radius)

            # Direction indicator
            if robot.path:
                nr, nc = robot.path[0]
                dr, dc = nr - r, nc - c
                ax = cx + dc * (radius - 2)
                ay = cy + dr * (radius - 2)
                pygame.draw.circle(self.screen, (255, 255, 255), (ax, ay), 3)

            # ID label
            id_txt = self.font_xs.render(str(robot.id), True, (255, 255, 255))
            id_rect = id_txt.get_rect(center=(cx, cy - 2))
            self.screen.blit(id_txt, id_rect)

            # Battery bar below robot
            bar_w = CELL_SIZE - 8
            bar_h = 4
            bar_x = c * CELL_SIZE + 4
            bar_y = r * CELL_SIZE + CELL_SIZE - 6
            ratio = max(robot.battery / BATTERY_MAX, 0)
            bg_rect = pygame.Rect(bar_x, bar_y, bar_w, bar_h)
            pygame.draw.rect(self.screen, (40, 40, 50), bg_rect)
            if ratio > 0.5:
                bar_color = (46, 204, 113)
            elif ratio > 0.25:
                bar_color = (241, 196, 15)
            else:
                bar_color = (231, 76, 60)
            fill_rect = pygame.Rect(bar_x, bar_y, int(bar_w * ratio), bar_h)
            pygame.draw.rect(self.screen, bar_color, fill_rect)

            # Status indicator
            status_colors = {
                "idle": (100, 100, 100),
                "moving": (46, 204, 113),
                "blocked": (231, 76, 60),
                "recalculating": (243, 156, 18),
                "charging": (52, 220, 220),
            }
            sc = status_colors.get(robot.status, (100, 100, 100))
            pygame.draw.circle(self.screen, sc,
                               (cx + radius - 2, cy + radius - 5), 3)

    # -- side panel -------------------------------------------------------
    def _draw_panel(self, stats, algorithm, move_interval, paused,
                    fleet, layout_id):
        panel_x = GRID_COLS * CELL_SIZE
        panel_rect = pygame.Rect(panel_x, 0, PANEL_WIDTH, WINDOW_HEIGHT)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, PANEL_ACCENT,
                         (panel_x, 0), (panel_x, WINDOW_HEIGHT), 2)

        x = panel_x + 14
        y = 12

        # Title
        title = self.font_lg.render("Factory Sim", True, PANEL_HEADER_COLOR)
        self.screen.blit(title, (x, y)); y += 28

        # Status
        state_text = "PAUSED" if paused else "RUNNING"
        state_color = (231, 76, 60) if paused else (46, 204, 113)
        st = self.font_md.render(state_text, True, state_color)
        self.screen.blit(st, (x, y)); y += 20

        # Algorithm + Layout + Speed
        self.screen.blit(self.font_sm.render(
            f"Algo: {algorithm.value}  Layout: {layout_id}/{LAYOUT_COUNT}",
            True, PANEL_ACCENT), (x, y)); y += 16
        self.screen.blit(self.font_sm.render(
            f"Speed: {MAX_MOVE_INTERVAL + 1 - move_interval}x",
            True, PANEL_TEXT_COLOR), (x, y)); y += 16

        # Overlay indicators
        ov = []
        if self.show_heatmap: ov.append("HEAT")
        if self.show_trails: ov.append("TRAIL")
        if ov:
            self.screen.blit(self.font_xs.render(
                "Overlays: " + "+".join(ov), True, (180, 140, 60)), (x, y))
        y += 16

        # Separator
        pygame.draw.line(self.screen, GRID_LINE_COLOR,
                         (x, y), (x + PANEL_WIDTH - 30, y)); y += 8

        # Stats
        if self.show_analytics and stats:
            lines = [
                f"Tick:        {stats['tick']}",
                f"Robots:      {stats['robots_total']}",
                f"  Active:    {stats['robots_active']}",
                f"  Idle:      {stats['robots_idle']}",
                f"  Blocked:   {stats['robots_blocked']}",
                f"  Charging:  {stats['robots_charging']}",
                f"Tasks Done:  {stats['tasks_completed']}",
                f"Recalcs:     {stats['recalculations']}",
                f"Steps:       {stats['total_steps']}",
                f"Avg Path:    {stats['avg_remaining_path']}",
                f"Avg Battery: {stats['avg_battery']}%",
                f"Collisions:  {stats['collisions_avoided']}",
            ]
            for line in lines:
                self.screen.blit(
                    self.font_xs.render(line, True, PANEL_TEXT_COLOR), (x, y))
                y += 14
            y += 4

        # Separator
        pygame.draw.line(self.screen, GRID_LINE_COLOR,
                         (x, y), (x + PANEL_WIDTH - 30, y)); y += 6

        # Task Queue
        self.screen.blit(self.font_md.render(
            f"Task Queue ({len(fleet.task_queue)})", True,
            PANEL_HEADER_COLOR), (x, y)); y += 16
        for i, (src, dst) in enumerate(fleet.task_queue[:5]):
            t = self.font_xs.render(
                f"  ({src[0]:02},{src[1]:02})->({dst[0]:02},{dst[1]:02})",
                True, (160, 160, 180))
            self.screen.blit(t, (x, y)); y += 12
        if len(fleet.task_queue) > 5:
            self.screen.blit(self.font_xs.render(
                f"  ...+{len(fleet.task_queue)-5} more", True,
                PANEL_TEXT_COLOR), (x, y)); y += 12
        y += 4

        # Separator
        pygame.draw.line(self.screen, GRID_LINE_COLOR,
                         (x, y), (x + PANEL_WIDTH - 30, y)); y += 6

        # Fleet details
        self.screen.blit(self.font_md.render(
            "Fleet", True, PANEL_HEADER_COLOR), (x, y)); y += 16
        for robot in fleet.robots:
            sym = {"idle":"o","moving":">","blocked":"x",
                   "recalculating":"~","charging":"+"}.get(robot.status, "?")
            bat = int(robot.battery)
            line = (f"#{robot.id}{sym} "
                    f"({robot.position[0]:02},{robot.position[1]:02}) "
                    f"bat:{bat}% d:{robot.tasks_completed}")
            self.screen.blit(
                self.font_xs.render(line, True, robot.color), (x, y))
            y += 13
            if y > WINDOW_HEIGHT - 120:
                more = len(fleet.robots) - fleet.robots.index(robot) - 1
                if more > 0:
                    self.screen.blit(self.font_xs.render(
                        f"  ...+{more} more", True, PANEL_TEXT_COLOR), (x, y))
                    y += 13
                break

        # Controls legend
        y = WINDOW_HEIGHT - 118
        pygame.draw.line(self.screen, GRID_LINE_COLOR,
                         (x, y), (x + PANEL_WIDTH - 30, y)); y += 5
        controls = [
            "LClick: wall  RClick: robot",
            "Space: pause  R: reset",
            "A/D: algorithm  +/-: speed",
            "1/2/3: layout",
            "H: heatmap  T: trails",
            "S: screenshot  E: export CSV",
            "Tab: toggle stats",
        ]
        for c in controls:
            self.screen.blit(
                self.font_xs.render(c, True, (110, 110, 130)), (x, y))
            y += 13

    # -- screenshot -------------------------------------------------------
    def save_screenshot(self):
        """Save current frame as PNG."""
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
        tick = pygame.time.get_ticks()
        path = os.path.join(SCREENSHOT_DIR, f"screenshot_{tick}.png")
        pygame.image.save(self.screen, path)
        self._screenshot_flash = 80
        return path

    # -- event handling ---------------------------------------------------
    def handle_events(self, floor, fleet, state: dict):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                state["running"] = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                grid_c = mx // CELL_SIZE
                grid_r = my // CELL_SIZE
                if 0 <= grid_r < GRID_ROWS and 0 <= grid_c < GRID_COLS:
                    if event.button == 1:
                        floor.toggle_wall(grid_r, grid_c)
                        for robot in fleet.robots:
                            if (grid_r, grid_c) in robot.path:
                                robot.recalculate(floor, state["algorithm"])
                    elif event.button == 3:
                        fleet.add_robot((grid_r, grid_c))

            # Click-and-drag to draw walls continuously
            elif event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    mx, my = event.pos
                    grid_c = mx // CELL_SIZE
                    grid_r = my // CELL_SIZE
                    if 0 <= grid_r < GRID_ROWS and 0 <= grid_c < GRID_COLS:
                        floor.place_obstacle(grid_r, grid_c)
                        for robot in fleet.robots:
                            if (grid_r, grid_c) in robot.path:
                                robot.recalculate(floor, state["algorithm"])

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    state["paused"] = not state["paused"]
                elif event.key == pygame.K_r:
                    state["reset"] = True
                elif event.key in (pygame.K_a, pygame.K_d):
                    if state["algorithm"] == Algorithm.ASTAR:
                        state["algorithm"] = Algorithm.DIJKSTRA
                    else:
                        state["algorithm"] = Algorithm.ASTAR
                    fleet.algorithm = state["algorithm"]
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS,
                                   pygame.K_KP_PLUS):
                    state["move_interval"] = max(
                        MIN_MOVE_INTERVAL, state["move_interval"] - 1)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    state["move_interval"] = min(
                        MAX_MOVE_INTERVAL, state["move_interval"] + 1)
                elif event.key == pygame.K_TAB:
                    self.show_analytics = not self.show_analytics
                elif event.key == pygame.K_h:
                    self.show_heatmap = not self.show_heatmap
                elif event.key == pygame.K_t:
                    self.show_trails = not self.show_trails
                elif event.key == pygame.K_s:
                    state["screenshot"] = True
                elif event.key == pygame.K_e:
                    state["export"] = True
                elif event.key == pygame.K_1:
                    state["layout_id"] = 1; state["reset"] = True
                elif event.key == pygame.K_2:
                    state["layout_id"] = 2; state["reset"] = True
                elif event.key == pygame.K_3:
                    state["layout_id"] = 3; state["reset"] = True

    def trigger_export_flash(self):
        self._export_flash = 60

    def quit(self):
        pygame.quit()
