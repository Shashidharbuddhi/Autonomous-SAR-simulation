"""Metrics and CSV/JSON output helpers."""
from __future__ import annotations
import csv, json, os

def summarize(robot, targets, total_explored, benchmark, output_dir):
    targets = [tuple(t) for t in targets]
    visited = set(robot.visited_targets)
    unreachable = [t for t in targets if t not in visited]
    result = {
        "grid_size": list(robot.true_map.shape),
        "start": list(robot.path_history[0]),
        "total_targets": len(targets),
        "targets_visited": len(visited),
        "target_coverage_percent": round(100 * len(visited) / max(1, len(targets)), 2),
        "unreachable_targets": [list(t) for t in unreachable],
        "path_length_grid_steps": max(0, len(robot.path_history)-1),
        "explored_cells_sum_per_plan": total_explored,
        "dynamic_obstacles_discovered": len(set(robot.discovered_obstacles)),
        "replanning_count": robot.replans,
        "benchmark_first_target": benchmark,
        "notes": "Runtime and exploration counts depend on hardware and map; this is a simulation."
    }
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    with open(os.path.join(output_dir, "path_log.csv"), "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "row", "column", "event"])
        for i, cell in enumerate(robot.path_history):
            writer.writerow([i, cell[0], cell[1], "start" if i == 0 else "move"])
        for event in robot.events:
            writer.writerow(["", "", "", json.dumps(event)])
    return result
