import numpy as np
from pypibt import path_length

def _is_straight_line_clear(grid, start, end, critical_section):
    """
    Checks if a straight line between two points is clear of obstacles.

    Args:
        grid (np.ndarray): 2D numpy array representing the grid.
        start (tuple): (x, y) tuple representing the start point.
        end (tuple): (x, y) tuple representing the end point.

    Returns:
        bool: True if the line is clear, False otherwise.
    """
    x1, y1 = start
    x2, y2 = end

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if x1 < 0 or x1 >= grid.shape[1] or y1 < 0 or y1 >= grid.shape[0] or not grid[y1, x1] or critical_section[y1, x1]:
            return False

        if x1 == x2 and y1 == y2:
            return True

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy


def straight_path(grid, path, critical_sections):
    """
    Strighten the path without running into obstacles. Essentially, reduce the number of jagged edges by
    checking if a straight line between two points is clear of obstacles.

    A.k.a "String pulling"
    """
    new_path = [(path[0][0], path[0][1], 0)]
    i = 0
    while i < len(path) - 1:
        j = i + 1
        critcal_section_check = critical_sections[i]
        while j < len(path) - 1 and _is_straight_line_clear(grid, path[i][:2], path[j][:2], critcal_section_check) and path[j][:2] != path[j-1][:2] and critical_sections[j][path[j][1], path[j][0]] == False:
            j += 1
            critcal_section_check = critical_sections[j] # The final velocity might be lower than the max velocity, so we need to check the critical sections of the entire path

        prev_pos = np.array(new_path[-1][:2]).astype(np.float64)
        cur_pos = np.array(path[j][:2]).astype(np.float64)
        diff = cur_pos - prev_pos
        length = np.linalg.norm(diff)
        step = diff / (j-i)
        if length > 0:
            for k in range(i, j):
                prev_pos += step
                new_path.append((prev_pos[0], prev_pos[1]))
        else:
            new_path.append((path[j][0], path[j][1], j))
        i = j
        
    return new_path


def identify_all_critical_regions(paths, grid):
    """
    Identifies critical regions in a path based on the density of agents

    Args:
        path (dict): dict of list of (x, y) tuples representing the path.
        grid (numpy.ndarray): 2D NumPy array representing obstacles (False for occupied).

    Returns:
        list: List of (numpy.ndarray) True if a cell is critical at a certain time, False otherwise.
    """
    critical_regions = []
    if len(paths) < 1:
        return critical_regions
    for t in range(len(paths[0])):
        critical_zone = np.full(grid.shape, False)

        agent_unsafe_zones = []
        for agent in paths:
            x1, y1 = paths[agent][t]
            x1 = int(x1)
            y1 = int(y1)
            to_mark1 = []
            for i in range(-1,2):
                for j in range(-1,2):
                    if x1+i >= 0 and x1+i < grid.shape[1] and y1+j >= 0 and y1+j < grid.shape[0] and grid[y1+j, x1+i]:
                        to_mark1.append((x1+i, y1+j))
            agent_unsafe_zones.append(set(to_mark1))

        for i in range(len(agent_unsafe_zones)):
            for j in range(i+1, len(agent_unsafe_zones)):
                if agent_unsafe_zones[i].intersection(agent_unsafe_zones[j]):
                    for coord in agent_unsafe_zones[i].union(agent_unsafe_zones[j]):
                        critical_zone[coord[1], coord[0]] = True
        critical_regions.append(critical_zone)
    return critical_regions

def smoothe_all_paths(agent_paths, obstacles):
    path_bef = 0
    path_aft = 0

    normal_paths = {}
    for agent in agent_paths:
        new_path = [(x[0], x[1]) for x in agent_paths[agent]]
        normal_paths[agent] = new_path
    critical_sections = identify_all_critical_regions(normal_paths, obstacles)

    for agent in agent_paths:
        new_path = [(x[0], x[1]) for x in agent_paths[agent]]
        path_bef += path_length(new_path)
        normalized = straight_path(obstacles, new_path, critical_sections)
        path_aft += path_length(normalized)
        normalized = [(x[0], x[1]) for x in normalized]
        agent_paths[agent] = normalized

    print("Path length before smoothing: ", path_bef)
    print("Path length after smoothing: ", path_aft)
    print("Path length reduction: ", path_bef - path_aft)
    print("Path length reduction percentage: ", (path_bef - path_aft) / path_bef * 100, "%")
    return agent_paths