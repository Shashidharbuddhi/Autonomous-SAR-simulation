"""BFS and A* for four-connected occupancy grids."""
from __future__ import annotations
from collections import deque
import heapq
import time

MOVES = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def valid_neighbors(grid, cell):
    r, c = cell
    rows, cols = grid.shape
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
            yield (nr, nc)

def reconstruct_path(parent, start, goal):
    if goal != start and goal not in parent:
        return []
    path, cur = [], goal
    while cur != start:
        path.append(cur)
        cur = parent[cur]
    path.append(start)
    path.reverse()
    return path

def bfs(grid, start, goal):
    """Returns (path, explored_count, runtime_seconds)."""
    start_time = time.perf_counter()
    if grid[start] or grid[goal]:
        return [], 0, time.perf_counter() - start_time
    q = deque([start])
    parent = {}
    seen = {start}
    explored = 0
    while q:
        cur = q.popleft()
        explored += 1
        if cur == goal:
            break
        for nxt in valid_neighbors(grid, cur):
            if nxt not in seen:
                seen.add(nxt)
                parent[nxt] = cur
                q.append(nxt)
    path = reconstruct_path(parent, start, goal) if goal == start or goal in parent else []
    return path, explored, time.perf_counter() - start_time

def astar(grid, start, goal):
    """Returns (path, explored_count, runtime_seconds), using Manhattan distance."""
    start_time = time.perf_counter()
    if grid[start] or grid[goal]:
        return [], 0, time.perf_counter() - start_time
    def h(p):
        return abs(p[0] - goal[0]) + abs(p[1] - goal[1])
    frontier = [(h(start), 0, start)]
    parent = {}
    g_cost = {start: 0}
    closed = set()
    explored = 0
    while frontier:
        _, g, cur = heapq.heappop(frontier)
        if cur in closed:
            continue
        closed.add(cur)
        explored += 1
        if cur == goal:
            break
        for nxt in valid_neighbors(grid, cur):
            tentative = g + 1
            if tentative < g_cost.get(nxt, float("inf")):
                g_cost[nxt] = tentative
                parent[nxt] = cur
                heapq.heappush(frontier, (tentative + h(nxt), tentative, nxt))
    path = reconstruct_path(parent, start, goal) if goal == start or goal in parent else []
    return path, explored, time.perf_counter() - start_time

def compare_algorithms(grid, start, goal):
    bpath, bexplored, btime = bfs(grid, start, goal)
    apath, aexplored, atime = astar(grid, start, goal)
    return {
        "start": list(start), "goal": list(goal),
        "bfs": {"reachable": bool(bpath), "path_length": max(0, len(bpath)-1),
                "explored_cells": bexplored, "runtime_seconds": btime},
        "astar": {"reachable": bool(apath), "path_length": max(0, len(apath)-1),
                  "explored_cells": aexplored, "runtime_seconds": atime},
    }
