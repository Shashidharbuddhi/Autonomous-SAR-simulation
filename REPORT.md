# Autonomous Search and Rescue Robot Simulation

## 1. Title
Autonomous Search and Rescue Robot Simulation using A*, BFS, Dynamic Obstacle Discovery, and OpenCV.

## 2. Problem Statement
After a disaster, some locations may be unsafe for human inspection. This project simulates a mobile robot that navigates a grid-based hazardous environment, avoids obstacles, searches designated locations, discovers previously unknown obstacles using a simulated local sensor, replans paths, and reports coverage and efficiency.

## 3. Objectives
1. Represent a hazard zone as a 2D occupancy grid.
2. Implement A* and BFS path-planning algorithms.
3. Prevent the robot from moving through obstacles.
4. Simulate local obstacle discovery and path replanning.
5. Convert a map image into an occupancy grid using OpenCV.
6. Measure path length, explored cells, target coverage, unreachable targets, and algorithm runtime.

## 4. Dataset / Simulation Inputs
The default environment is a deterministic, synthetic 24×32 occupancy grid generated in `environment.py`. A value of 0 means free space and 1 means obstacle. The robot starts at (1, 1), and six target cells are defined in `TARGETS`. Static obstacles are known at startup. Dynamic obstacles are withheld from the initial known map and are revealed when they fall inside the simulated sensor footprint. This is a custom simulation input, not an external dataset.

An optional image can be supplied with `--map-image`. OpenCV converts it to grayscale, resizes it to the grid dimensions, and classifies pixels below intensity 127 as obstacles and brighter pixels as free space. Therefore, the input image must use dark obstacles on a light background.

## 5. Methodology
1. Construct the true map and the robot's initially known map.
2. Use four-connected movement (up, down, left, right), with unit cost per move.
3. Use BFS as a baseline and A* with Manhattan-distance heuristic as the primary planner.
4. Move one cell at a time. After each move, simulate a local sensor that reveals nearby obstacles.
5. Replan with A* from the robot's current position whenever navigation continues with updated map information.
6. If no route exists in the current known map, mark the target unreachable for that attempt.
7. Save plots, a path log, and JSON metrics.

## 6. Algorithms
A* evaluates nodes using f(n) = g(n) + h(n), where g is the distance from the start and h is Manhattan distance to the target. BFS explores cells in layers and returns a shortest path when all movements have equal cost. Both use four-neighbour connectivity and disallow diagonal movement.

## 7. OpenCV Image Processing
The optional image pipeline loads a grayscale image, resizes it to the occupancy-grid dimensions, then applies a threshold. Dark pixels become obstacles (1); light pixels become free cells (0). The output grid can be exported as a PNG for inspection. This is image-based map conversion; it is not object detection from a live camera.

## 8. Performance Metrics
- Path length: number of movement steps in the robot's path history.
- Explored cells: number of cells expanded during path planning; counts from multiple planning attempts may be summed.
- Target coverage: visited targets / total targets × 100.
- Unreachable targets: targets not successfully visited.
- Replanning count: number of route-planning attempts during robot navigation.
- Runtime: measured independently for BFS and A* in the first-target benchmark.

## 9. Results
Run `python main.py` and copy the actual console metrics from `outputs/metrics.json` into this section. Do not invent performance results. Include `outputs/initial_map.png`, `outputs/final_run.png`, and a small table comparing BFS and A* for at least three start-goal pairs for a stronger report.

## 10. Limitations
- The environment is synthetic and grid-based.
- The simulated sensor has a square sensing footprint and perfect obstacle classification.
- The robot has no physical dynamics, wheel slip, localization error, or real camera feed.
- The image thresholding assumes dark obstacles and a light background.
- Performance depends on map layout and hardware; one test does not establish general superiority.

## 11. Conclusion
The simulation demonstrates grid representation, autonomous route planning, obstacle avoidance, target tracking, dynamic obstacle discovery, replanning, image-to-grid processing, and quantitative evaluation. It provides a foundation for later work with realistic sensor noise, SLAM, camera-based perception, or a robotics simulator.

## 12. References
1. Hart, P. E., Nilsson, N. J., and Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths.
2. OpenCV documentation, Image Processing: https://docs.opencv.org/
3. Python documentation: https://docs.python.org/3/
4. NumPy documentation: https://numpy.org/doc/
5. Matplotlib documentation: https://matplotlib.org/stable/
