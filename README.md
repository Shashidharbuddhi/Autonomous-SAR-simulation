# Autonomous Search and Rescue Robot Simulation

A Python grid-world simulation demonstrating A* and BFS path planning, target coverage,
dynamic obstacle discovery, replanning, and optional OpenCV image-to-occupancy-grid conversion.

## Requirements
- Python 3.10+
- Packages in `requirements.txt`

## Install
```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
```

## Run
```bash
python main.py
```
For a faster non-animated run:
```bash
python main.py --no-animation
```
Use a map image as the occupancy grid:
```bash
python main.py --map-image data/map.png --no-animation
```
White/light pixels are free; dark pixels are obstacles. The image is resized to the grid size.
The default sample map is generated from documented, deterministic simulation inputs in `environment.py`.

Outputs are saved in `outputs/`:
- `initial_map.png`: occupancy grid and start/targets
- `final_run.png`: final discovered map and robot path
- `metrics.json`: run metrics and BFS/A* benchmark
- `path_log.csv`: robot path coordinates and event labels

## Map conventions
- `0` = free cell
- `1` = obstacle
- coordinates are `(row, column)`, starting at `(0, 0)` in the top-left
- four-neighbour movement only; diagonal moves are not allowed
- each movement costs 1 grid unit
- target coverage = visited targets / all targets × 100%
- dynamic obstacle cells are hidden from the robot's initial known map and are revealed when they fall within the simulated sensor radius
- this is a software simulation, not a real robot or validated disaster-response system
