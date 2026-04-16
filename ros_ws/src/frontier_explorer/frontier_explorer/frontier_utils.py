from collections import deque
import math

import numpy as np


# =========================
# CONFIGURATION
# =========================
UNKNOWN = -1
FREE_THRESHOLD = 10     # <= this is considered free
MIN_FRONTIER_SIZE = 8   # reject small frontiers
MAX_FRONTIER_SIZE = 600  # stop growing oversized frontiers

# Choose connectivity (IMPORTANT: be consistent everywhere)
USE_8_CONNECTED = True


def is_free_value(value):
    """Return True when an occupancy value is treated as free."""
    return 0 <= value <= FREE_THRESHOLD


def is_free_cell(grid, x, y):
    """Return True when the grid cell is free enough for traversal."""
    return is_free_value(grid[y, x])


def get_neighbors(x, y, width, height):
    """Return valid neighbors (4 or 8 connected)."""
    directions_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    directions_8 = directions_4 + [
        (-1, -1), (-1, 1),
        (1, -1), (1, 1)
    ]

    directions = directions_8 if USE_8_CONNECTED else directions_4

    neighbors = []
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height:
            neighbors.append((nx, ny))

    return neighbors


# =========================
# 1. IS FRONTIER CELL
# =========================
def is_frontier_cell(grid, x, y, width, height):
    """Return True when a free cell borders at least one unknown cell."""
    if not is_free_cell(grid, x, y):
        return False

    for nx, ny in get_neighbors(x, y, width, height):
        if grid[ny, nx] == UNKNOWN:
            return True

    return False


# =========================
# 2. GROW FRONTIER (BFS CLUSTER)
# =========================
def grow_frontier(grid, start, visited, width, height):
    """Grow one connected frontier cluster with BFS."""
    frontier = []
    queue = deque()
    queue.append(start)
    visited.add(start)

    while queue:
        x, y = queue.popleft()
        frontier.append((x, y))

        if len(frontier) >= MAX_FRONTIER_SIZE:
            break

        for nx, ny in get_neighbors(x, y, width, height):

            if (nx, ny) in visited:
                continue

            if is_frontier_cell(grid, nx, ny, width, height):
                visited.add((nx, ny))
                queue.append((nx, ny))

    return frontier


# =========================
# 3. FIND FRONTIERS (MAIN BFS SEARCH)
# =========================
def find_frontiers(map_msg, robot_cell):
    """Find reachable frontier clusters with BFS over free space."""
    width = map_msg.info.width
    height = map_msg.info.height

    grid = np.array(map_msg.data).reshape((height, width))

    visited = set()
    frontier_visited = set()

    queue = deque()
    queue.append(robot_cell)
    visited.add(robot_cell)

    frontiers = []

    while queue:
        x, y = queue.popleft()

        for nx, ny in get_neighbors(x, y, width, height):

            if (nx, ny) in visited:
                continue

            # Only expand through free space
            if grid[ny, nx] >= 0 and grid[ny, nx] <= FREE_THRESHOLD:
                visited.add((nx, ny))
                queue.append((nx, ny))

            # Check if frontier
            if is_frontier_cell(grid, nx, ny, width, height):

                if (nx, ny) in frontier_visited:
                    continue

                # Grow full frontier cluster
                frontier = grow_frontier(
                    grid,
                    (nx, ny),
                    frontier_visited,
                    width,
                    height
                )

                # Reject small frontiers
                if len(frontier) >= MIN_FRONTIER_SIZE:
                    frontiers.append(frontier)

    return frontiers


# =========================
# OPTIONAL HELPERS
# =========================
def frontier_centroid(frontier):
    """Compute centroid of a frontier cluster."""
    xs = [p[0] for p in frontier]
    ys = [p[1] for p in frontier]

    return int(sum(xs) / len(xs)), int(sum(ys) / len(ys))


def world_to_grid(map_msg, wx, wy):
    """Convert world coordinates into a map cell."""
    info = map_msg.info
    mx = math.floor((wx - info.origin.position.x) / info.resolution)
    my = math.floor((wy - info.origin.position.y) / info.resolution)

    if 0 <= mx < info.width and 0 <= my < info.height:
        return int(mx), int(my)

    return None


def grid_to_world(map_msg, mx, my):
    """Convert a map cell into the center of that cell in world space."""
    info = map_msg.info
    wx = info.origin.position.x + (mx + 0.5) * info.resolution
    wy = info.origin.position.y + (my + 0.5) * info.resolution
    return wx, wy
