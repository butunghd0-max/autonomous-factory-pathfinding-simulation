"""
config.py — Global constants for the Factory Pathfinding Simulation.
"""

from enum import Enum

# ── Grid Dimensions ──────────────────────────────────────────────────────────
GRID_ROWS = 20
GRID_COLS = 30
CELL_SIZE = 32

# ── Cell Types ───────────────────────────────────────────────────────────────
EMPTY = 0
WALL = 1
STATION_LOAD = 2
STATION_DELIVER = 3
DYNAMIC_OBSTACLE = 4

# ── Algorithm Enum ───────────────────────────────────────────────────────────
class Algorithm(Enum):
    ASTAR = "A*"
    DIJKSTRA = "Dijkstra"

# ── Timing ───────────────────────────────────────────────────────────────────
FPS = 30
BASE_MOVE_INTERVAL = 6          # ticks between each robot step (lower = faster)
MIN_MOVE_INTERVAL = 1
MAX_MOVE_INTERVAL = 20

# ── UI Layout ────────────────────────────────────────────────────────────────
PANEL_WIDTH = 280
WINDOW_WIDTH = GRID_COLS * CELL_SIZE + PANEL_WIDTH
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE

# ── Colors (dark theme) ─────────────────────────────────────────────────────
BG_COLOR          = (18,  18,  24)
GRID_LINE_COLOR   = (38,  38,  50)
EMPTY_COLOR       = (30,  30,  42)
WALL_COLOR        = (70,  70,  85)
STATION_LOAD_CLR  = (46, 204, 113)    # green
STATION_DELIVER_CLR = (231, 76, 60)   # red
DYNAMIC_OBS_COLOR = (243, 156,  18)   # amber
PATH_ALPHA        = 100               # transparency for path trails

# Robot palette — each robot gets a colour from this cycle
ROBOT_COLORS = [
    (52,  152, 219),   # blue
    (155,  89, 182),   # purple
    (26,  188, 156),   # teal
    (241, 196,  15),   # yellow
    (230, 126,  34),   # orange
    (46,  204, 113),   # green
    (236, 112,  99),   # salmon
    (93,  173, 226),   # light blue
]

PANEL_BG          = (24,  24,  34)
PANEL_TEXT_COLOR   = (200, 200, 210)
PANEL_ACCENT       = (52,  152, 219)
PANEL_HEADER_COLOR = (255, 255, 255)

# ── Heatmap gradient (low traffic → high traffic) ───────────────────────────
HEATMAP_COLORS = [
    (30,  30,  60),    # cold  (barely visited)
    (20,  80, 120),
    (20, 150, 130),
    (80, 200,  80),
    (200, 200,  40),
    (240, 160,  20),
    (230,  80,  30),
    (200,  30,  30),    # hot   (heavily visited)
]
TRAIL_DECAY = 0.92              # trail opacity multiplier per tick (0-1)
TRAIL_MAX_LENGTH = 40           # max remembered positions per robot

# ── Layout Count ─────────────────────────────────────────────────────────────
LAYOUT_COUNT = 3                # number of factory layouts (keys 1-3)

# ── Fleet Defaults ───────────────────────────────────────────────────────────
INITIAL_ROBOT_COUNT = 5
AUTO_DISPATCH_INTERVAL = 60    # ticks between auto-dispatch attempts
