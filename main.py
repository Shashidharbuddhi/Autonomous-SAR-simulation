
"""Autonomous Search and Rescue Robot Simulation.

Runs a complete rescue mission, compares BFS and A*,
and exports evaluation metrics and visualizations.
"""

import argparse
import os
import time

from environment import (
    START,
    TARGETS,
    build_true_map,
    build_known_map,
)
from path_planning import compare_algorithms
from robot import RescueRobot
from visualization import plot_map
from vision import save_grid_as_image
from evaluation import summarize


def run_simulation(output_dir="outputs", algorithm="A*"):
    """Execute a rescue mission and save its results."""
    os.makedirs(output_dir, exist_ok=True)

    # 1. Build the environment.
    true_map = build_true_map()
    known_map = build_known_map(true_map)

    print("=" * 60)
    print("AUTONOMOUS SEARCH AND RESCUE ROBOT")
    print("=" * 60)
    print(f"Grid size       : {true_map.shape[0]} x {true_map.shape[1]}")
    print(f"Starting cell   : {START}")
    print(f"Rescue targets  : {len(TARGETS)}")
    print(f"Selected planner: {algorithm}")
    print()

    # 2. Compare BFS and A* on the same initial map and endpoints.
    benchmark = compare_algorithms(
        known_map.copy(),
        START,
        TARGETS[0],
    )

    print("BFS vs A* benchmark (first target)")
    print("-" * 60)

    for name, key in (("BFS", "bfs"), ("A*", "astar")):
        result = benchmark[key]

        print(f"{name}:")
        print(f"  Reachable       : {result['reachable']}")
        print(f"  Path length     : {result['path_length']}")
        print(f"  Explored cells  : {result['explored_cells']}")
        print(
            f"  Runtime         : "
            f"{result['runtime_seconds'] * 1000:.4f} ms"
        )

    print()

    # 3. Save the initial environment visualization.
    save_grid_as_image(
        true_map,
        os.path.join(output_dir, "initial_map.png"),
    )

    # 4. Initialize the robot.
    robot = RescueRobot(
        true_map=true_map,
        known_map=known_map,
        start=START,
        sensor_radius=2,
        algorithm=algorithm,
    )

    # 5. Navigate to each rescue target.
    mission_start = time.perf_counter()

    for index, target in enumerate(TARGETS, start=1):
        print(f"Target {index}/{len(TARGETS)}: {target}")

        if target in robot.visited_targets:
            print("  Already visited.")
            continue

        if target in robot.unreachable_targets:
            print("  Previously determined unreachable.")
            continue

        reached = robot.navigate_to(target)

        if reached:
            print(f"  Reached. Robot position: {robot.position}")
        else:
            print("  Target unreachable with current map knowledge.")

        print(
            f"  Discovered obstacles: "
            f"{len(robot.discovered_obstacles)}"
        )

    mission_runtime = time.perf_counter() - mission_start

    # 6. Calculate the mission metrics.
    total_explored = robot.total_explored

    metrics = summarize(
        robot=robot,
        targets=TARGETS,
        total_explored=total_explored,
        benchmark=benchmark,
        output_dir=output_dir,
    )

    metrics["selected_algorithm"] = algorithm
    metrics["mission_runtime_seconds"] = mission_runtime

    # Save the extra mission runtime and selected algorithm.
    import json

    with open(
        os.path.join(output_dir, "metrics.json"),
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(metrics, file, indent=2)

    # 7. Save the final visualization.
        plot_map(
        grid=true_map,
        start=START,
        targets=TARGETS,
        path=robot.path_history,
        discovered=robot.discovered_obstacles,
        title=f"Final Rescue Mission - {algorithm}",
        output_path=os.path.join(output_dir, "final_run.png"),
    )

    # 8. Print the final report.
    print()
    print("=" * 60)
    print("MISSION SUMMARY")
    print("=" * 60)
    print(
        f"Targets visited : "
        f"{len(robot.visited_targets)}/{len(TARGETS)}"
    )
    print(
        f"Target coverage : "
        f"{metrics['target_coverage_percent']:.2f}%"
    )
    print(
        f"Path length     : "
        f"{metrics['path_length_grid_steps']} grid steps"
    )
    print(f"Planning calls  : {robot.planning_calls}")
    print(f"Explored cells  : {robot.total_explored}")
    print(
        f"Planning time   : "
        f"{robot.total_planning_time * 1000:.4f} ms"
    )
    print(
        f"Mission runtime : "
        f"{mission_runtime:.4f} seconds"
    )
    print(
        f"Obstacles found : "
        f"{len(robot.discovered_obstacles)}"
    )
    print(f"Replan events   : {robot.replans}")

    if metrics["unreachable_targets"]:
        print(
            f"Unvisited targets: "
            f"{metrics['unreachable_targets']}"
        )

    print()
    print(f"Results saved to: {os.path.abspath(output_dir)}")
    print("Generated files:")
    print("  - initial_map.png")
    print("  - final_run.png")
    print("  - metrics.json")
    print("  - path_log.csv")
    print("  - benchmark_comparison.csv")
    print("=" * 60)

    return metrics


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Search and Rescue Robot Simulation"
    )

    parser.add_argument(
        "--algorithm",
        choices=["A*", "BFS"],
        default="A*",
        help="Path-planning algorithm for the rescue mission.",
    )

    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory for saved metrics and visualizations.",
    )

    args = parser.parse_args()

    run_simulation(
        output_dir=args.output_dir,
        algorithm=args.algorithm,
    )


if __name__ == "__main__":
    main()
