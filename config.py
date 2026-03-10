"""
config.py -- Global constants for the Factory Pathfinding Simulation.
"""

from enum import Enum

# -- Grid Dimensions --------------------------------------------------------
GRID_ROWS = 20
GRID_COLS = 30
CELL_SIZE = 32

# -- Cell Types -------------------------------------------------------------
EMPTY = 0
WALL = 1
STATION_LOAD = 2
STATION_DELIVER = 3
DYNAMIC_OBSTACLE = 4
SLOW_ZONE = 5            # weighted terrain -- costs more to traverse
CHARGE_STATION = 6        # battery recharge point
ONE_WAY_UP = 7
ONE_WAY_DOWN = 8
ONE_WAY_LEFT = 9
ONE_WAY_RIGHT = 10

# Cell traversal costs (used by pathfinding)
CELL_COST = {
    EMPTY: 1,
    STATION_LOAD: 1,
    STATION_DELIVER: 1,
    SLOW_ZONE: 3,
    CHARGE_STATION: 1,
    ONE_WAY_UP: 1,
    ONE_WAY_DOWN: 1,
    ONE_WAY_LEFT: 1,
    ONE_WAY_RIGHT: 1,
}

# One-way direction vectors (cell_type -> allowed (dr, dc) entering direction)
ONE_WAY_DIRS = {
    ONE_WAY_UP:    (-1, 0),
    ONE_WAY_DOWN:  (1, 0),
    ONE_WAY_LEFT:  (0, -1),
    ONE_WAY_RIGHT: (0, 1),
}

# -- Algorithm Enum ---------------------------------------------------------
class Algorithm(Enum):
    ASTAR = "A*"
    DIJKSTRA = "Dijkstra"

# -- Timing -----------------------------------------------------------------
FPS = 30
BASE_MOVE_INTERVAL = 6
MIN_MOVE_INTERVAL = 1
MAX_MOVE_INTERVAL = 20

# -- UI Layout --------------------------------------------------------------
PANEL_WIDTH = 280
WINDOW_WIDTH = GRID_COLS * CELL_SIZE + PANEL_WIDTH
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE

# -- Colors (dark theme) ----------------------------------------------------
BG_COLOR            = (18,  18,  24)
GRID_LINE_COLOR     = (38,  38,  50)
EMPTY_COLOR         = (30,  30,  42)
WALL_COLOR          = (70,  70,  85)
STATION_LOAD_CLR    = (46, 204, 113)
STATION_DELIVER_CLR = (231, 76,  60)
DYNAMIC_OBS_COLOR   = (243, 156,  18)
SLOW_ZONE_COLOR     = (80,  60,  100)
CHARGE_STATION_CLR  = (52, 220, 220)
ONE_WAY_COLOR       = (60,  80,  50)

PATH_ALPHA = 100

ROBOT_COLORS = [
    (52,  152, 219),
    (155,  89, 182),
    (26,  188, 156),
    (241, 196,  15),
    (230, 126,  34),
    (46,  204, 113),
    (236, 112,  99),
    (93,  173, 226),
]

PANEL_BG            = (24,  24,  34)
PANEL_TEXT_COLOR     = (200, 200, 210)
PANEL_ACCENT        = (52,  152, 219)
PANEL_HEADER_COLOR  = (255, 255, 255)

# -- Heatmap gradient -------------------------------------------------------
HEATMAP_COLORS = [
    (30,  30,  60),
    (20,  80, 120),
    (20, 150, 130),
    (80, 200,  80),
    (200, 200,  40),
    (240, 160,  20),
    (230,  80,  30),
    (200,  30,  30),
]
TRAIL_DECAY = 0.92
TRAIL_MAX_LENGTH = 40

# -- Layout Count -----------------------------------------------------------
LAYOUT_COUNT = 3

# -- Fleet Defaults ---------------------------------------------------------
INITIAL_ROBOT_COUNT = 5
AUTO_DISPATCH_INTERVAL = 60

# -- Battery ----------------------------------------------------------------
BATTERY_MAX = 100.0
BATTERY_DRAIN_PER_STEP = 1.0
BATTERY_DRAIN_SLOW_ZONE = 2.0
BATTERY_CHARGE_RATE = 5.0
BATTERY_LOW_THRESHOLD = 25.0

# -- Congestion -------------------------------------------------------------
CONGESTION_WEIGHT = 0.5
CONGESTION_RADIUS = 2

# -- Deadlock ---------------------------------------------------------------
DEADLOCK_TIMEOUT = 12      # ticks blocked before trying an alternate route
DEADLOCK_JITTER = 3.0      # random cost added during deadlock reroute

# -- Moving Obstacles -------------------------------------------------------
NUM_MOVING_OBSTACLES = 3
MOVING_OBS_INTERVAL = 4    # ticks between obstacle movements

# -- Pathfinding Visualization ----------------------------------------------
EXPLORED_ASTAR_CLR   = (52, 152, 219, 50)    # blue tint
EXPLORED_DIJKSTRA_CLR = (231, 76, 60, 50)    # red tint

# -- Export -----------------------------------------------------------------
SCREENSHOT_DIR = "exports"
