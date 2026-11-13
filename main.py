
"""Main entry point for the Autonomous Search and Rescue Robot."""

from __future__ import annotations

import argparse
import csv
import json
import os
import time

from environment import START, TARGETS, build_true_map, build_known_map
from path_planning import compare_algorithms
from robot import RescueRobot
from visualization import plot_map
from evaluation import summarize


def benchmark_all_targets(known_map, start, targets, output_dir):
    """Compare BFS and A* from the start cell to every target."""

    print("\nBFS vs A* benchmark (all targets)")
    print("-" * 65)

    rows = []
    first_target_benchmark = None

    for index, target in enumerate(targets, start=1):
        result = compare_algorithms(known_map.copy(), start, target)

        if index == 1:
            first_target_benchmark = result

        print(f"\nTarget {index}/{len(targets)}: {target}")

        for algorithm_name in ("bfs", "astar"):
            metrics = result[algorithm_name]

            print(
                f"  {algorithm_name.upper():5} | "
                f"Reachable: {metrics['reachable']} | "
                f"Path: {metrics['path_length']} steps | "
                f"Explored: {metrics['explored_cells']} | "
                f"Time: {metrics['runtime_seconds'] * 1000:.4f} ms"
            )

            rows.append({
                "target_number": index,
                "target_row": target[0],
                "target_column": target[1],
                "algorithm": algorithm_name.upper(),
                "reachable": metrics["reachable"],
                "path_length": metrics["path_length"],
                "explored_cells": metrics["explored_cells"],
                "runtime_ms": round(
                    metrics["runtime_seconds"] * 1000, 6
                ),
            })

    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(
        output_dir, "all_targets_benchmark.csv"
    )

    fieldnames = [
        "target_number",
        "target_row",
        "target_column",
        "algorithm",
        "reachable",
        "path_length",
        "explored_cells",
        "runtime_ms",
    ]

    with open(output_path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nAll-target benchmark saved to: {output_path}")

    return first_target_benchmark


def run_simulation(output_dir="outputs", algorithm="A*"):
    """Run the complete rescue mission and export results."""

    mission_start_time = time.perf_counter()

    print("=" * 60)
    print("AUTONOMOUS SEARCH AND RESCUE ROBOT")
    print("=" * 60)

    true_map = build_true_map()
    known_map = build_known_map(true_map)

    print(f"Grid size       : {true_map.shape[0]} x {true_map.shape[1]}")
    print(f"Starting cell   : {START}")
    print(f"Rescue targets  : {len(TARGETS)}")
    print(f"Selected planner: {algorithm}")

    os.makedirs(output_dir, exist_ok=True)

    # Save the initial environment map.
   

    # Benchmark both algorithms against every target.
    benchmark = benchmark_all_targets(
        known_map=known_map,
        start=START,
        targets=TARGETS,
        output_dir=output_dir,
    )

    # Start the actual mission with the selected planner.
    robot = RescueRobot(
        true_map=true_map,
        known_map=known_map,
        start=START,
        algorithm=algorithm,
    )

    print("\n" + "=" * 60)
    print("RESCUE MISSION")
    print("=" * 60)

    for index, target in enumerate(TARGETS, start=1):
        print(f"\nTarget {index}/{len(TARGETS)}: {target}")

        success = robot.navigate_to(target)

        if success:
            print(f"  Reached. Robot position: {robot.position}")
        else:
            print(f"  Could not reach target. Robot position: {robot.position}")

        print(
            f"  Discovered obstacles: "
            f"{len(robot.discovered_obstacles)}"
        )

    # Mission runtime excludes the all-target benchmark.
    # The value is measured around the mission separately below
    # in the updated implementation using the robot's planning totals.
    mission_runtime = time.perf_counter() - mission_start_time

    # Export metrics and movement/event logs.
    total_explored = robot.total_explored

    metrics = summarize(
        robot=robot,
        targets=TARGETS,
        total_explored=total_explored,
        benchmark=benchmark,
        output_dir=output_dir,
    )

    metrics["selected_algorithm"] = algorithm
    metrics["mission_runtime_seconds"] = round(mission_runtime, 6)

    metrics_path = os.path.join(output_dir, "metrics.json")
    with open(metrics_path, "w", encoding="utf-8") as file:
        json.dump(metrics, file, indent=2)

    # Save the final route and discovered obstacles.
    final_map_path = os.path.join(output_dir, "final_run.png")

    plot_map(
        grid=true_map,
        start=START,
        targets=TARGETS,
        path=robot.path_history,
        discovered=robot.discovered_obstacles,
        title=f"Final Rescue Mission - {algorithm}",
        output_path=final_map_path,
    )

    print("\n" + "=" * 60)
    print("MISSION SUMMARY")
    print("=" * 60)
    print(f"Targets visited : {len(robot.visited_targets)}/{len(TARGETS)}")
    print(f"Target coverage : {metrics['target_coverage_percent']:.2f}%")
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
    print(f"Mission runtime : {mission_runtime:.4f} seconds")
    print(f"Obstacles found : {len(robot.discovered_obstacles)}")
    print(f"Replan events   : {robot.replans}")

    print(f"\nResults saved to: {os.path.abspath(output_dir)}")
    print("Generated files:")
    print("  - initial_map.png")
    print("  - final_run.png")
    print("  - metrics.json")
    print("  - path_log.csv")
    print("  - benchmark_comparison.csv")
    print("  - all_targets_benchmark.csv")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description="Autonomous Search and Rescue Robot Simulation"
    )

    parser.add_argument(
        "--algorithm",
        choices=["A*", "BFS"],
        default="A*",
        help="Path-planning algorithm for the rescue mission",
    )

    parser.add_argument(
        "--output-dir",
        default="outputs",
        help="Directory for generated images and evaluation files",
    )

    args = parser.parse_args()

    run_simulation(
        output_dir=args.output_dir,
        algorithm=args.algorithm,
    )


if __name__ == "__main__":
    main()
