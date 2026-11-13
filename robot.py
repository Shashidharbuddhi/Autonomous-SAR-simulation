
"""Rescue robot with limited-range sensing and stepwise navigation."""

from __future__ import annotations

from path_planning import astar


class RescueRobot:
    def __init__(self, true_map, known_map, start, sensor_radius=2):
        self.true_map = true_map
        self.known_map = known_map.copy()

        self.position = tuple(start)
        self.sensor_radius = sensor_radius

        self.path_history = [tuple(start)]
        self.visited_targets = []
        self.unreachable_targets = []

        self.planning_calls = 0
        self.replans = 0

        self.discovered_obstacles = []
        self.events = []

        self.sense()

    def sense(self):
        """Discover obstacles inside the robot's square sensor range."""
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

    def step_toward(self, target):
        """
        Advance by at most one cell toward a target.

        Returns one of:
        'moved', 'reached', 'unreachable', or 'replanned'.
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

        path, _, _ = astar(
            self.known_map,
            self.position,
            target,
        )
        self.planning_calls += 1

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

        # Safety check against the actual environment.
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

        # Move exactly one cell.
        self.position = next_cell
        self.path_history.append(self.position)

        # Sense immediately after moving.
        previous_discoveries = len(self.discovered_obstacles)
        self.sense()

        # Count a replan-triggering discovery separately from
        # ordinary movement and ordinary path-planning calls.
        if len(self.discovered_obstacles) > previous_discoveries:
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
        """Navigate to a target using repeated one-cell steps."""
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
