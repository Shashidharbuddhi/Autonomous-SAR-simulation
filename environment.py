"""Occupancy-grid environment and documented deterministic scenario."""
from __future__ import annotations
import numpy as np

ROWS, COLS = 24, 32
START = (1, 1)
TARGETS = [(2, 27), (10, 5), (18, 27), (21, 4), (12, 15), (5, 30)]
# Static obstacles are visible at startup. Dynamic obstacles are unknown until sensed.
STATIC_OBSTACLES = [
    (r, c) for r in range(4, 20) for c in [9, 10]
    if r not in (7, 8, 15, 16)
] + [
    (r, c) for r in range(2, 22) for c in [20, 21]
    if r not in (4, 5, 11, 12, 18, 19)
] + [
    (6, c) for c in range(12, 19)
] + [
    (15, c) for c in range(12, 19) if c not in (14, 15, 16)
]
DYNAMIC_OBSTACLES = {(3, 25), (4, 25), (5, 25), (13, 12), (14, 12), (15, 12), (19, 26), (20, 26)}

def build_true_map():
    grid = np.zeros((ROWS, COLS), dtype=np.uint8)
    for cell in STATIC_OBSTACLES + list(DYNAMIC_OBSTACLES):
        grid[cell] = 1
    # Ensure targets and start remain free.
    grid[START] = 0
    for t in TARGETS:
        grid[t] = 0
    return grid

def build_known_map(true_map, map_image=None):
    if map_image is not None:
        # image import is handled in vision.py; supplied image is treated as a fully known map.
        from vision import image_to_occupancy_grid
        return image_to_occupancy_grid(map_image, true_map.shape)
    known = true_map.copy()
    for cell in DYNAMIC_OBSTACLES:
        known[cell] = 0
    return known
