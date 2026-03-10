"""
main.py -- Entry point for the Autonomous Factory Pathfinding Simulation.

Controls
--------
  Left-click   : Toggle wall / dynamic obstacle
  Right-click  : Add a new robot
  Space        : Pause / Resume
  R            : Reset simulation
  A / D        : Switch algorithm (A* / Dijkstra)
  + / -        : Speed up / slow down
  1 / 2 / 3   : Switch factory layout
  H            : Toggle heatmap overlay
  T            : Toggle robot trails
  S            : Save screenshot (PNG)
  E            : Export heatmap data (CSV)
  Tab          : Toggle analytics panel
"""

from config import Algorithm, BASE_MOVE_INTERVAL, INITIAL_ROBOT_COUNT
from environment import FactoryFloor
from robot import FleetManager, AGV
from renderer import Renderer
from analytics import SimulationAnalytics


def create_simulation(layout_id: int = 1):
    """Initialise all simulation components."""
    floor = FactoryFloor(layout_id=layout_id)
    fleet = FleetManager(floor, Algorithm.ASTAR)
    fleet.spawn_initial(INITIAL_ROBOT_COUNT)
    fleet.enqueue_tasks(8)     # seed the task queue
    fleet.auto_dispatch()
    analytics = SimulationAnalytics()
    renderer = Renderer()
    return floor, fleet, analytics, renderer


def main():
    floor, fleet, analytics, renderer = create_simulation()

    state = {
        "running": True,
        "paused": False,
        "algorithm": Algorithm.ASTAR,
        "move_interval": BASE_MOVE_INTERVAL,
        "reset": False,
        "layout_id": 1,
        "screenshot": False,
        "export": False,
    }

    tick = 0

    while state["running"]:
        # -- handle reset -------------------------------------------------
        if state["reset"]:
            AGV._id_counter = 0
            layout = state.get("layout_id", 1)
            floor, fleet, analytics, _ = create_simulation(layout)
            state["reset"] = False
            state["paused"] = False
            fleet.algorithm = state["algorithm"]
            tick = 0

        # -- handle screenshot / export -----------------------------------
        if state.get("screenshot"):
            renderer.save_screenshot()
            state["screenshot"] = False
        if state.get("export"):
            analytics.export_heatmap_csv()
            renderer.trigger_export_flash()
            state["export"] = False

        # -- events -------------------------------------------------------
        renderer.handle_events(floor, fleet, state)

        # -- simulation step ----------------------------------------------
        if not state["paused"]:
            tick += 1
            if tick % state["move_interval"] == 0:
                fleet.step_all()
                analytics.update(fleet.robots)

        # -- render -------------------------------------------------------
        summary = analytics.get_summary(fleet.robots)
        renderer.draw(floor, fleet, summary,
                      state["algorithm"], state["move_interval"],
                      state["paused"], analytics_obj=analytics)

    renderer.quit()


if __name__ == "__main__":
    main()
