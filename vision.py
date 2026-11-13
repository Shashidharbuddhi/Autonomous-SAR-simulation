"""OpenCV image-to-occupancy-grid conversion."""
from __future__ import annotations
import cv2
import numpy as np

def image_to_occupancy_grid(image_path, shape, threshold=127):
    """Convert an image to grid: light pixels=free (0), dark pixels=obstacle (1)."""
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Could not read map image: {image_path}")
    rows, cols = shape
    resized = cv2.resize(image, (cols, rows), interpolation=cv2.INTER_AREA)
    # White/light = free; black/dark = obstacle.
    return (resized < threshold).astype(np.uint8)

def save_grid_as_image(grid, output_path, cell_size=20):
    """Export occupancy grid as a readable black-and-white PNG."""
    image = np.where(grid == 1, 0, 255).astype(np.uint8)
    image = cv2.resize(image, (grid.shape[1]*cell_size, grid.shape[0]*cell_size),
                       interpolation=cv2.INTER_NEAREST)
    if not cv2.imwrite(str(output_path), image):
        raise IOError(f"Could not write image: {output_path}")
