
"""Evaluation metrics and CSV/JSON export helpers."""

from __future__ import annotations

import csv
import json
import os


def export_benchmark_csv(benchmark, output_dir):
    """Save BFS and A* benchmark results as a CSV file."""
    os.makedirs(output_dir, exist_ok=True)

    output_path = os.path.join(
        output_dir,
        "benchmark_comparison.csv",
    )

    rows = []

    for algorithm in ("bfs", "astar"):
        result = benchmark.get(algorithm, {})

        rows.append({
            "algorithm": algorithm.upper(),
            "start": str(benchmark.get("start", "")),
            "goal": str(benchmark.get("goal", "")),
            "reachable": result.get("reachable", False),
            "path_length": result.get("path_length", 0),
            "explored_cells": result.get("explored_cells", 0),
            "runtime_ms": round(
                result.get("runtime_seconds", 0) * 1000,
                6,
            ),
        })

    with open(
        output_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        fieldnames = [
            "algorithm",
            "start",
            "goal",
            "reachable",
            "path_length",
            "explored_cells",
            "runtime_ms",
        ]

        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
        )
        writer.writeheader()
        writer.writerows(rows)

    return output_path


def summarize(robot, targets, total_explored, benchmark, output_dir):
    """Create mission metrics, JSON output, and CSV logs."""
    targets = [tuple(target) for target in targets]

    visited = set(robot.visited_targets)
    unreachable = [
        target
        for target in targets
        if target not in visited
    ]

    total_targets = len(targets)

    result = {
        "grid_size": list(robot.true_map.shape),
        "start": list(robot.path_history[0]),
        "total_targets": total_targets,
        "targets_visited": len(visited),
        "target_coverage_percent": round(
            100 * len(visited) / max(1, total_targets),
            2,
        ),
        "unreachable_targets": [
            list(target) for target in unreachable
        ],
        "path_length_grid_steps": max(
            0, len(robot.path_history) - 1
        ),
        "explored_cells_sum_per_plan": total_explored,
        "dynamic_obstacles_discovered": len(
            set(robot.discovered_obstacles)
        ),
        "replanning_count": robot.replans,
        "benchmark_first_target": benchmark,
        "notes": (
            "Runtime depends on hardware and system load. "
            "Explored-cell totals may count the same cell in "
            "multiple planning calls. Replanning count records "
            "events tracked by the robot, not total planning calls."
        ),
    }

    # Include additional statistics when supported by the robot.
    if hasattr(robot, "planning_calls"):
        result["planning_calls"] = robot.planning_calls

    if hasattr(robot, "total_explored"):
        result["robot_total_explored_cells"] = robot.total_explored

    if hasattr(robot, "total_planning_time"):
        result["robot_total_planning_time_ms"] = round(
            robot.total_planning_time * 1000,
            6,
        )

    os.makedirs(output_dir, exist_ok=True)

    # Export the complete mission summary.
    metrics_path = os.path.join(output_dir, "metrics.json")

    with open(
        metrics_path,
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(result, file, indent=2)

    # Export the robot's movement history and event log.
    path_log_path = os.path.join(output_dir, "path_log.csv")

    with open(
        path_log_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.writer(file)

        writer.writerow([
            "step",
            "row",
            "column",
            "event",
        ])

        for step, cell in enumerate(robot.path_history):
            writer.writerow([
                step,
                cell[0],
                cell[1],
                "start" if step == 0 else "move",
            ])

        for event in robot.events:
            writer.writerow([
                "",
                "",
                "",
                json.dumps(event),
            ])

    # Export the algorithm benchmark in a spreadsheet-friendly format.
    export_benchmark_csv(benchmark, output_dir)

    return result
