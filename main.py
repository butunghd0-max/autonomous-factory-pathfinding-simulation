"""
main.py -- Entry point for the Autonomous Factory Pathfinding Simulation.
"""

from config import Algorithm, BASE_MOVE_INTERVAL, INITIAL_ROBOT_COUNT
from environment import FactoryFloor, MovingObstacleManager
from robot import FleetManager, AGV
from renderer import Renderer
from analytics import SimulationAnalytics
from pathfinding import compare_algorithms


def create_simulation(layout_id: int = 1):
    floor = FactoryFloor(layout_id=layout_id)
    fleet = FleetManager(floor, Algorithm.ASTAR)
    fleet.spawn_initial(INITIAL_ROBOT_COUNT)
    fleet.enqueue_tasks(8)
    fleet.auto_dispatch()
    analytics = SimulationAnalytics()
    renderer = Renderer()
    moving_obs = MovingObstacleManager(floor)
    return floor, fleet, analytics, renderer, moving_obs


def _run_comparison(floor, fleet):
    """Pick a robot with a destination and compare A*/Dijkstra on its route."""
    for robot in fleet.robots:
        if robot.destination:
            return compare_algorithms(
                floor, robot.position, robot.destination,
                fleet._congestion_map)
    load = floor.get_stations(2)
    deliver = floor.get_stations(3)
    if load and deliver:
        return compare_algorithms(floor, load[0], deliver[0],
                                  fleet._congestion_map)
    return None


def main():
    floor, fleet, analytics, renderer, moving_obs = create_simulation()

    state = {
        "running": True,
        "paused": False,
        "algorithm": Algorithm.ASTAR,
        "move_interval": BASE_MOVE_INTERVAL,
        "reset": False,
        "layout_id": 1,
        "screenshot": False,
        "export": False,
        "run_comparison": False,
    }

    tick = 0

    while state["running"]:
        if state["reset"]:
            AGV._id_counter = 0
            layout = state.get("layout_id", 1)
            floor, fleet, analytics, _, moving_obs = create_simulation(layout)
            state["reset"] = False
            state["paused"] = False
            fleet.algorithm = state["algorithm"]
            renderer.selected_robot_id = None
            renderer._comparison_data = None
            tick = 0

        if state.get("screenshot"):
            renderer.save_screenshot()
            state["screenshot"] = False
        if state.get("export"):
            analytics.export_heatmap_csv()
            analytics.export_robot_stats_csv(fleet.robots)
            renderer.trigger_export_flash()
            state["export"] = False

        if state.get("run_comparison"):
            data = _run_comparison(floor, fleet)
            renderer.set_comparison_data(data)
            state["run_comparison"] = False

        renderer.handle_events(floor, fleet, state)

        if not state["paused"]:
            tick += 1
            if tick % state["move_interval"] == 0:
                robot_positions = {r.position for r in fleet.robots}
                moving_obs.step(floor, robot_positions)
                obs_positions = moving_obs.get_positions()
                fleet.step_all(moving_obs_positions=obs_positions)
                analytics.update(fleet.robots)

        summary = analytics.get_summary(fleet.robots)
        renderer.draw(floor, fleet, summary,
                      state["algorithm"], state["move_interval"],
                      state["paused"], analytics_obj=analytics,
                      moving_obs_mgr=moving_obs)

    renderer.quit()


if __name__ == "__main__":
    main()
