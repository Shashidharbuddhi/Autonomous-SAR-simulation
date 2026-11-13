
"""Rescue robot with selectable BFS/A* path planning."""

from __future__ import annotations

from path_planning import astar, bfs


class RescueRobot:
    def __init__(
        self,
        true_map,
        known_map,
        start,
        sensor_radius=2,
        algorithm="A*",
    ):
        self.true_map = true_map
        self.known_map = known_map.copy()

        self.position = tuple(start)
        self.sensor_radius = sensor_radius
        self.algorithm = algorithm.upper()

        if self.algorithm not in ("A*", "BFS"):
            raise ValueError("Algorithm must be 'A*' or 'BFS'.")

        self.path_history = [tuple(start)]
        self.visited_targets = []
        self.unreachable_targets = []

        self.planning_calls = 0
        self.replans = 0

        self.discovered_obstacles = []
        self.events = []

        # Statistics for the most recent path-planning call.
        self.last_explored = 0
        self.last_planning_time = 0.0

        # Cumulative planning statistics.
        self.total_explored = 0
        self.total_planning_time = 0.0

        self.sense()

    def set_algorithm(self, algorithm):
        """Change the planner used for future movement steps."""
        algorithm = algorithm.upper()

        if algorithm not in ("A*", "BFS"):
            raise ValueError("Algorithm must be 'A*' or 'BFS'.")

        self.algorithm = algorithm

        self.events.append({
            "event": "algorithm_changed",
            "algorithm": algorithm,
        })

    def sense(self):
        """Discover obstacles inside the robot's sensor footprint."""
        r, c = self.position
        rows, cols = self.true_map.shape

        for rr in range(
            max(0, r - self.sensor_radius),
            min(rows, r + self.sensor_radius + 1),
        ):
            for cc in range(
                max(0, c - self.sensor_radius),
                min(cols, c + self.sensor_radius + 1),
            ):
                if (
                    self.true_map[rr, cc] == 1
                    and self.known_map[rr, cc] == 0
                ):
                    self.known_map[rr, cc] = 1
                    cell = (rr, cc)

                    if cell not in self.discovered_obstacles:
                        self.discovered_obstacles.append(cell)

                        self.events.append({
                            "event": "obstacle_discovered",
                            "cell": [rr, cc],
                        })

    def _plan_path(self, target):
        """Run the currently selected path-planning algorithm."""
        planner = bfs if self.algorithm == "BFS" else astar

        path, explored, elapsed = planner(
            self.known_map,
            self.position,
            target,
        )

        self.planning_calls += 1
        self.last_explored = explored
        self.last_planning_time = elapsed

        self.total_explored += explored
        self.total_planning_time += elapsed

        return path

    def step_toward(self, target):
        """
        Move at most one cell toward a target.

        Returns: 'moved', 'reached', 'unreachable', or 'replanned'.
        """
        target = tuple(target)

        if target in self.visited_targets:
            return "reached"

        if target in self.unreachable_targets:
            return "unreachable"

        if self.position == target:
            self.visited_targets.append(target)
            self.events.append({
                "event": "target_visited",
                "target": list(target),
            })
            return "reached"

        path = self._plan_path(target)

        if not path:
            self.unreachable_targets.append(target)
            self.events.append({
                "event": "target_unreachable",
                "target": list(target),
            })
            return "unreachable"

        if len(path) < 2:
            self.visited_targets.append(target)
            self.events.append({
                "event": "target_visited",
                "target": list(target),
            })
            return "reached"

        next_cell = path[1]

        # Safety guard: never enter a cell blocked in the true map.
        if self.true_map[next_cell] == 1:
            if self.known_map[next_cell] == 0:
                self.known_map[next_cell] = 1

                if next_cell not in self.discovered_obstacles:
                    self.discovered_obstacles.append(next_cell)

                self.events.append({
                    "event": "blocked_step_prevented",
                    "cell": list(next_cell),
                })

            self.replans += 1
            return "replanned"

        # Move one cell.
        self.position = next_cell
        self.path_history.append(self.position)

        # Discover nearby obstacles after moving.
        previous_count = len(self.discovered_obstacles)
        self.sense()

        if len(self.discovered_obstacles) > previous_count:
            self.replans += 1

        if self.position == target:
            self.visited_targets.append(target)
            self.events.append({
                "event": "target_visited",
                "target": list(target),
            })
            return "reached"

        return "moved"

    def navigate_to(self, target):
        """Navigate to a target using repeated single-cell steps."""
        target = tuple(target)

        if target in self.visited_targets:
            return True

        if target in self.unreachable_targets:
            return False

        while self.position != target:
            result = self.step_toward(target)

            if result == "unreachable":
                return False

        return target in self.visited_targets
 