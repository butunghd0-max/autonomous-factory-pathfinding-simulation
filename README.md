# Autonomous Factory Pathfinding Simulation

A Python-based simulation of autonomous guided vehicles (AGVs) navigating a factory floor. The project implements the A\* and Dijkstra pathfinding algorithms to calculate optimal routes across a 2D grid, while handling fleet coordination, dynamic obstacles, and real-time collision avoidance. The simulation is rendered using Pygame.

This project was built as a way to better understand how graph traversal algorithms work in a practical, applied context. I am still learning many of these concepts, so the code reflects my current understanding of pathfinding, object-oriented design, and simulation architecture. There are likely areas that could be improved or optimized further.

![Simulation Screenshot](screenshot.png)

---

## Table of Contents

1. [Features](#features)
2. [Getting Started](#getting-started)
3. [Controls](#controls)
4. [Project Structure](#project-structure)
5. [How It Works](#how-it-works)
6. [Algorithms](#algorithms)
7. [Technologies Used](#technologies-used)
8. [References and Acknowledgements](#references-and-acknowledgements)

---

## Features

- **A\* and Dijkstra Pathfinding** -- both algorithms are implemented and can be toggled at runtime to compare their behavior on the same factory layout.
- **Three Factory Layouts** -- press 1, 2, or 3 to switch between a standard factory floor with machine clusters, a warehouse with parallel shelving rows, and an open-plan layout with scattered pillars.
- **Fleet Management** -- multiple AGVs are auto-dispatched between loading docks and delivery stations, simulating a basic logistics workflow.
- **Collision Avoidance** -- a priority-based system where lower-ID robots claim cells first; other robots wait rather than overlap.
- **Dynamic Obstacles** -- users can click on the grid to place or remove walls during the simulation, forcing robots to recalculate their paths in real time.
- **Traffic Heatmap** -- an overlay that color-codes each cell by how frequently it has been visited, making it easy to identify bottleneck corridors. Toggle with H.
- **Robot Trails** -- each robot leaves a fading trail of dots showing where it has recently been, helpful for visualizing movement patterns. Toggle with T.
- **Live Analytics Panel** -- a side panel displays running statistics including tasks completed, total steps, recalculations triggered, and collisions avoided.
- **Adjustable Speed** -- the simulation speed can be increased or decreased with keyboard controls.

---

## Getting Started

### Prerequisites

- Python 3.10 or later
- `pip` package manager

### Installation

```bash
cd "autonomous-factory-pathfinding-simulation"
pip install -r requirements.txt
```

### Running the Simulation

```bash
python main.py
```

---

## Controls

| Key or Action       | Effect                                                |
| ------------------- | ----------------------------------------------------- |
| Left-click on grid  | Toggle a wall (dynamic obstacle) on or off            |
| Right-click on grid | Spawn a new robot at the clicked cell                 |
| Space               | Pause or resume the simulation                        |
| R                   | Reset the entire simulation to its initial state      |
| A or D              | Switch between A\* and Dijkstra algorithms            |
| + or -              | Increase or decrease simulation speed                 |
| 1, 2, or 3          | Switch factory layout (Factory, Warehouse, Open Plan) |
| H                   | Toggle traffic heatmap overlay                        |
| T                   | Toggle robot trail visualization                      |
| Tab                 | Show or hide the analytics panel                      |

---

## Project Structure

```
autonomous-factory-pathfinding-simulation/
    main.py              -- Entry point; initializes the simulation and runs the game loop
    config.py            -- Global constants including grid size, colors, timing, and cell types
    environment.py       -- FactoryFloor class; manages the 2D grid, obstacles, and station positions
    pathfinding.py       -- A* and Dijkstra implementations with a shared find_path() interface
    robot.py             -- AGV class and FleetManager class; handles movement, dispatch, and collisions
    renderer.py          -- Pygame-based rendering, UI panel drawing, and user input event handling
    analytics.py         -- SimulationAnalytics class; tracks and exposes live simulation metrics
    requirements.txt     -- Python package dependencies (pygame, numpy)
    README.md            -- This file
```

---

## How It Works

### The Environment

The factory floor is represented as a 20-by-30 grid stored in a 2D NumPy array. Each cell in the grid holds an integer value that indicates what occupies that space:

| Value | Meaning                                               |
| ----- | ----------------------------------------------------- |
| 0     | Navigable aisle (open space a robot can move through) |
| 1     | Static wall or machinery (permanently blocked)        |
| 2     | Loading dock (where robots pick up materials)         |
| 3     | Delivery station (where robots drop off materials)    |
| 4     | Dynamic obstacle (placed by the user at runtime)      |

There are three built-in layouts, each designed to test the pathfinding under different conditions:

1. **Factory** (default) -- machine clusters, conveyor belt walls with gaps, vertical partitions, and pillar obstacles. Loading docks on the left, delivery stations on the right.
2. **Warehouse** -- parallel shelving rows with periodic cross-aisle gaps and a central vertical corridor. Stations along the top and bottom edges.
3. **Open Plan** -- mostly open floor with scattered 2x2 pillar blocks and two L-shaped assembly zones in opposite corners. Stations on left and right edges.

This grid-based representation is essentially a graph structure. Each walkable cell is a node, and it connects to its four immediate neighbors (up, down, left, right). This is the same underlying data structure used in geographic routing and map software, just applied to a factory context instead of streets and intersections.

### Heatmap and Trails

Two visual overlays help analyze robot behavior:

- **Traffic heatmap (H)** -- the analytics module counts every cell visit. The renderer maps these counts to an 8-step color gradient from cold (dark blue, barely visited) to hot (deep red, heavily visited). This makes it easy to spot bottleneck aisles where many robots converge.
- **Robot trails (T)** -- each robot stores up to 40 recent positions. The renderer draws these as fading dots in the robot's color, with older positions more transparent. This is useful for seeing the actual paths robots took, as opposed to the planned paths shown ahead of them.

### Pathfinding

Both A\* and Dijkstra share the same `find_path()` function. The only difference is whether a heuristic is used:

- **A\*** uses the Manhattan distance formula as its heuristic, which estimates how far a node is from the target based on horizontal and vertical distance only.
- **Dijkstra** sets the heuristic to zero, so it explores nodes in all directions equally without any guidance toward the goal.

Internally, both algorithms use a min-heap priority queue (Python's `heapq` module) to always expand the most promising node first. A parent-pointer dictionary tracks which node was visited from where, so the final path can be reconstructed by walking backward from the destination to the start.

I chose to implement both algorithms side by side because it helped me understand the role of the heuristic more clearly. Watching Dijkstra explore many more nodes than A\* on the same grid made the theoretical difference very concrete.

### Fleet Management

The `FleetManager` class coordinates all robots in the simulation:

1. **Auto-dispatch** -- idle robots are periodically assigned new tasks. If a robot is at a loading dock, it is sent to a delivery station, and vice versa.
2. **Priority stepping** -- each simulation tick, robots are processed in order of their ID. Lower-ID robots move first and claim their target cell, preventing higher-ID robots from stepping into the same space.
3. **Collision avoidance** -- if a robot's next cell is already claimed by another robot, the robot waits in place until the cell becomes available.
4. **Dynamic recalculation** -- if the user places a wall on a cell that lies along a robot's planned path, the robot immediately recalculates a new route from its current position using the active algorithm.

This system is simplified compared to real-world fleet management, which would involve more complex scheduling, traffic flow optimization, and deadlock resolution. But it was a useful exercise in thinking about how multiple agents share a constrained space.

---

## Algorithms

### A\* Search

A\* is a best-first search algorithm that finds the shortest path between two nodes on a graph. It evaluates each node using the formula:

```
f(n) = g(n) + h(n)
```

Where:

- `g(n)` is the exact cost from the start node to the current node (in this simulation, each step costs 1).
- `h(n)` is the heuristic estimate of the cost from the current node to the goal. This project uses the Manhattan distance: `|x1 - x2| + |y1 - y2|`.
- `f(n)` is the total estimated cost. The algorithm always processes the node with the lowest `f(n)` first.

The Manhattan distance heuristic is admissible for a grid restricted to four-directional movement, which means it never overestimates the true distance. This guarantees that A\* will find the optimal (shortest) path.

I learned about A\* primarily through the pseudocode on the Wikipedia article and the Red Blob Games tutorial, both of which were very helpful in understanding how the open and closed sets work and why the heuristic matters.

### Dijkstra's Algorithm

Dijkstra's algorithm is a foundational shortest-path algorithm that works by expanding outward from the start node in all directions. It is essentially A\* with `h(n) = 0`, meaning it has no guidance toward the goal and instead explores every reachable node in order of increasing distance.

While Dijkstra is guaranteed to find the shortest path, it visits significantly more nodes than A* on most grids because it lacks the directional guidance of a heuristic. In this simulation, you can see this difference clearly by toggling between the two algorithms and observing how much faster A* resolves the path.

Dijkstra's algorithm was originally described by Edsger Dijkstra in 1959 and remains one of the most fundamental algorithms in computer science for solving shortest-path problems on weighted graphs.

---

## Technologies Used

- **Python 3** -- the primary programming language for all simulation logic and object-oriented design.
- **Pygame** -- used for real-time 2D rendering, drawing the grid and robots, and handling user input events like mouse clicks and keyboard presses.
- **NumPy** -- used to store and manipulate the 2D factory grid efficiently as a numerical array.
- **heapq (Python standard library)** -- provides the min-heap priority queue used by both A\* and Dijkstra to efficiently select the next node to process.

---

## References and Acknowledgements

The following sources were used during the development of this project. They helped me understand the algorithms, libraries, and concepts involved. Listed in MLA 9 format, in alphabetical order.

Cormen, Thomas H., et al. _Introduction to Algorithms_. 4th ed., MIT Press, 2022.

Dijkstra, Edsger W. "A Note on Two Problems in Connexion with Graphs." _Numerische Mathematik_, vol. 1, no. 1, 1959, pp. 269-271. Springer, https://doi.org/10.1007/BF01386390. Accessed 8 Mar. 2026.

Hart, Peter E., et al. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." _IEEE Transactions on Systems Science and Cybernetics_, vol. 4, no. 2, 1968, pp. 100-107. IEEE, https://doi.org/10.1109/TSSC.1968.300136. Accessed 8 Mar. 2026.

Hunter, John D. "Matplotlib: A 2D Graphics Environment." _Computing in Science and Engineering_, vol. 9, no. 3, 2007, pp. 90-95. IEEE, https://doi.org/10.1109/MCSE.2007.55. Accessed 9 Mar. 2026.

Lague, Sebastian. "A* Pathfinding (E01: Algorithm Explanation)." *YouTube\*, uploaded by Sebastian Lague, 24 Feb. 2018, https://www.youtube.com/watch?v=-L-WgKMFuhE. Accessed 9 Mar. 2026.

LaValle, Steven M. _Planning Algorithms_. Cambridge University Press, 2006.

NumPy Developers. "NumPy Documentation." _NumPy_, 2024, https://numpy.org/doc/stable/. Accessed 9 Mar. 2026.

Patel, Amit. "Introduction to A*." *Red Blob Games\*, 2014, https://www.redblobgames.com/pathfinding/a-star/introduction.html. Accessed 8 Mar. 2026.

Patel, Amit. "Implementation of A*." *Red Blob Games\*, 2014, https://www.redblobgames.com/pathfinding/a-star/implementation.html. Accessed 8 Mar. 2026.

Pygame Community. "Pygame Documentation." _Pygame_, 2024, https://www.pygame.org/docs/. Accessed 9 Mar. 2026.

Python Software Foundation. "heapq -- Heap Queue Algorithm." _Python 3.12 Documentation_, 2024, https://docs.python.org/3/library/heapq.html. Accessed 9 Mar. 2026.

Python Software Foundation. "The Python Tutorial." _Python 3.12 Documentation_, 2024, https://docs.python.org/3/tutorial/. Accessed 8 Mar. 2026.

Russell, Stuart, and Peter Norvig. _Artificial Intelligence: A Modern Approach_. 4th ed., Pearson, 2021.

Skiena, Steven S. _The Algorithm Design Manual_. 3rd ed., Springer, 2020.

Wikipedia contributors. "A* Search Algorithm." *Wikipedia, The Free Encyclopedia*, Wikimedia Foundation, 2024, https://en.wikipedia.org/wiki/A*\_search_algorithm. Accessed 8 Mar. 2026.

Wikipedia contributors. "Dijkstra's Algorithm." _Wikipedia, The Free Encyclopedia_, Wikimedia Foundation, 2024, https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm. Accessed 8 Mar. 2026.

Wikipedia contributors. "Manhattan Distance." _Wikipedia, The Free Encyclopedia_, Wikimedia Foundation, 2024, https://en.wikipedia.org/wiki/Taxicab_geometry. Accessed 8 Mar. 2026.

Wikipedia contributors. "Priority Queue." _Wikipedia, The Free Encyclopedia_, Wikimedia Foundation, 2024, https://en.wikipedia.org/wiki/Priority_queue. Accessed 9 Mar. 2026.
