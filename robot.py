"""Robot simulation with limited-range sensing and replanning."""
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
        self.replans = 0
        self.discovered_obstacles = []
        self.events = []
        self.sense()

    def sense(self):
        """Reveal true-map cells within a square sensor footprint."""
        r, c = self.position
        rows, cols = self.true_map.shape
        for rr in range(max(0, r-self.sensor_radius), min(rows, r+self.sensor_radius+1)):
            for cc in range(max(0, c-self.sensor_radius), min(cols, c+self.sensor_radius+1)):
                if self.true_map[rr, cc] == 1 and self.known_map[rr, cc] == 0:
                    self.known_map[rr, cc] = 1
                    self.discovered_obstacles.append((rr, cc))
                    self.events.append({"event": "obstacle_discovered", "cell": [rr, cc]})

    def navigate_to(self, target):
        target = tuple(target)
        while self.position != target:
            path, _, _ = astar(self.known_map, self.position, target)
            if not path:
                self.events.append({"event": "target_unreachable", "target": list(target)})
                return False
            self.replans += 1
            # Take one step at a time so new sensor data can trigger replanning.
            next_cell = path[1] if len(path) > 1 else target
            if self.true_map[next_cell] == 1:
                # Safety guard: should not happen when sensing/replanning is active.
                self.known_map[next_cell] = 1
                self.discovered_obstacles.append(next_cell)
                self.events.append({"event": "blocked_step_prevented", "cell": list(next_cell)})
                continue
            self.position = next_cell
            self.path_history.append(self.position)
            self.sense()
        self.visited_targets.append(target)
        self.events.append({"event": "target_visited", "target": list(target)})
        return True
