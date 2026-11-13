"""Run the autonomous search and rescue simulation."""
from __future__ import annotations
import argparse, json, os
from environment import START, TARGETS, build_true_map, build_known_map
from path_planning import astar, compare_algorithms
from robot import RescueRobot
from visualization import plot_map
from vision import save_grid_as_image

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-image", help="Optional map image: light pixels free, dark pixels blocked")
    parser.add_argument("--no-animation", action="store_true", help="Compatibility flag; simulation is non-interactive by default")
    parser.add_argument("--sensor-radius", type=int, default=2)
    args = parser.parse_args()
    root = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(root, "outputs")
    os.makedirs(out, exist_ok=True)

    true_map = build_true_map()
    known_map = build_known_map(true_map, args.map_image)
    # If a custom image is used, use its grid as ground truth and disable hidden dynamic obstacles.
    if args.map_image:
        true_map = known_map.copy()

    plot_map(known_map, START, TARGETS, title="Initial Known Occupancy Grid",
             output_path=os.path.join(out, "initial_map.png"))
    save_grid_as_image(known_map, os.path.join(out, "occupancy_grid.png"))

    benchmark = compare_algorithms(known_map, START, TARGETS[0])
    robot = RescueRobot(true_map, known_map, START, args.sensor_radius)
    total_explored = 0
    print("Autonomous Search and Rescue Robot Simulation")
    print(f"Grid: {true_map.shape[0]} x {true_map.shape[1]} | Start: {START} | Targets: {len(TARGETS)}")
    print("BFS vs A* on initial map to first target:")
    print(json.dumps(benchmark, indent=2))
    for i, target in enumerate(TARGETS, start=1):
        if robot.position == tuple(target):
            robot.visited_targets.append(tuple(target))
            continue
        path, explored, _ = astar(robot.known_map, robot.position, tuple(target))
        total_explored += explored
        ok = robot.navigate_to(target)
        print(f"Target {i} {target}: {'VISITED' if ok else 'UNREACHABLE'}")
    # Report unreachable targets in final discovered map.
    final_grid = robot.known_map
    plot_map(final_grid, START, TARGETS, path=robot.path_history,
             discovered=robot.discovered_obstacles,
             title="Final Discovered Map and Robot Path",
             output_path=os.path.join(out, "final_run.png"))
    from evaluation import summarize
    result = summarize(robot, TARGETS, total_explored, benchmark, out)
    print("\nFinal metrics:")
    print(json.dumps(result, indent=2))
    print(f"\nOutputs saved to: {out}")

if __name__ == "__main__":
    main()
