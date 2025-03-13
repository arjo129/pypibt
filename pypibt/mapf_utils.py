import os
import re
from typing import TypeAlias

import numpy as np

# y, x
Grid: TypeAlias = np.ndarray
Coord: TypeAlias = tuple[int, int]
Config: TypeAlias = list[Coord]
Configs: TypeAlias = list[Config]


def get_grid(map_file: str) -> Grid:
    width, height = 0, 0
    with open(map_file, "r") as f:
        # retrieve map size
        for row in f:
            # get width
            res = re.match(r"width\s(\d+)", row)
            if res:
                width = int(res.group(1))

            # get height
            res = re.match(r"height\s(\d+)", row)
            if res:
                height = int(res.group(1))

            if width > 0 and height > 0:
                break

        # retrieve map
        grid = np.zeros((height, width), dtype=bool)
        y = 0
        for row in f:
            row = row.strip()
            if len(row) == width and row != "map":
                grid[y] = [s == "." for s in row]
                y += 1

    # simple error check
    assert y == height, f"map format seems strange, check {map_file}"

    # grid[y, x] -> True: available, False: obstacle
    return grid


def get_scenario(scen_file: str, N: int | None = None) -> tuple[Config, Config]:
    with open(scen_file, "r") as f:
        starts, goals = [], []
        for row in f:
            res = re.match(
                r"\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+", row
            )
            if res:
                x_s, y_s, x_g, y_g = [int(res.group(k)) for k in range(1, 5)]
                starts.append((y_s, x_s))  # align with grid
                goals.append((y_g, x_g))

                # check the number of agents
                if (N is not None) and len(starts) >= N:
                    break

    return starts, goals


def is_valid_coord(grid: Grid, coord: Coord) -> bool:
    y, x = coord
    if y < 0 or y >= grid.shape[0] or x < 0 or x >= grid.shape[1] or not grid[coord]:
        return False
    return True


def get_neighbors(grid: Grid, coord: Coord) -> list[Coord]:
    # coord: y, x
    neigh: list[Coord] = []

    # check valid input
    if not is_valid_coord(grid, coord):
        return neigh

    y, x = coord

    if x > 0 and grid[y, x - 1]:
        neigh.append((y, x - 1))

    if x < grid.shape[1] - 1 and grid[y, x + 1]:
        neigh.append((y, x + 1))

    if y > 0 and grid[y - 1, x]:
        neigh.append((y - 1, x))

    if y < grid.shape[0] - 1 and grid[y + 1, x]:
        neigh.append((y + 1, x))

    # Diagonal step. comment if you don't want to use diagonal steps.
    # TODO also check the adjecent cells for diagonal steps
    
    if x > 0 and y > 0 and grid[y - 1, x - 1] and grid[y - 1, x] and grid[y, x - 1]:
        neigh.append((y - 1, x - 1))

    if x > 0 and y < grid.shape[0] - 1 and grid[y + 1, x - 1] and grid[y + 1, x] and grid[y, x - 1]:
        neigh.append((y + 1, x - 1))

    if x < grid.shape[1] - 1 and y < grid.shape[0] - 1 and grid[y + 1, x + 1] and grid[y + 1, x] and grid[y, x + 1]:
        neigh.append((y + 1, x + 1))

    if x < grid.shape[1] - 1 and y > 0 and grid[y - 1, x + 1] and grid[y - 1, x] and grid[y, x + 1]:
        neigh.append((y - 1, x + 1))

    return neigh


def save_configs_for_visualizer(configs: Configs, filename: str) -> None:
    dirname = os.path.dirname(filename)
    if len(dirname) > 0:
        os.makedirs(dirname, exist_ok=True)
    with open(filename, "w") as f:
        for t, config in enumerate(configs):
            row = f"{t}:" + "".join([f"({x},{y})," for (y, x) in config]) + "\n"
            f.write(row)


def validate_mapf_solution(
    grid: Grid,
    starts: Config,
    goals: Config,
    solution: Configs,
) -> None:
    # starts
    assert all(
        [u == v for (u, v) in zip(starts, solution[0])]
    ), "invalid solution, check starts"

    # goals
    assert all(
        [u == v for (u, v) in zip(goals, solution[-1])]
    ), "invalid solution, check goals"

    T = len(solution)
    N = len(starts)

    for t in range(T):
        for i in range(N):
            v_i_now = solution[t][i]
            v_i_pre = solution[max(t - 1, 0)][i]

            # check continuity
            assert v_i_now in [v_i_pre] + get_neighbors(
                grid, v_i_pre
            ), "invalid solution, check connectivity"

            # check collision
            for j in range(i + 1, N):
                v_j_now = solution[t][j]
                v_j_pre = solution[max(t - 1, 0)][j]
                assert not (v_i_now == v_j_now), "invalid solution, vertex collision"
                assert not (
                    v_i_now == v_j_pre and v_i_pre == v_j_now
                ), "invalid solution, edge collision"


def is_valid_mapf_solution(
    grid: Grid,
    starts: Config,
    goals: Config,
    solution: Configs,
) -> bool:
    try:
        validate_mapf_solution(grid, starts, goals, solution)
        return True
    except Exception as e:
        print(e)
        return False

def cost_of_solution(starts: Config,
                  solution: Configs) -> int:
    T = len(solution)
    N = len(starts)
    cost = 0
    for t in range(T):
        for i in range(N):
            v_i_now = solution[t][i]
            v_i_pre = solution[max(t - 1, 0)][i]
            if v_i_now != v_i_pre:
                cost += 1
    return cost

def get_critical_areas(solution: Configs, radius: int) -> list[list[int]]:
    """
    Go through both configurations and check if there are any places where 
    two agents are within the radius of each other. If they are, add the
    coordinates to a list of critical intersections.
    """
    for sol_id1 in range(len(solution)):
        sol1_crit = []
        for sol_id2 in range(len(solution)):
            critical_intersections = []
            for t in range(len(solution[sol_id1])):
                if abs(solution[sol_id1][t][0] - solution[sol_id2][t][0]) <= radius and abs(solution[sol_id1][t][1] - solution[sol_id2][t][1]) <= radius:
                    critical_intersections.append(t)
            sol1_crit.append(critical_intersections)
    return sol1_crit

def get_non_critical_segment(solution: Config, critical: list[int]) -> list[list[tuple[int, int]]]:
    """
    Get list of non-critical segments from a solution
    """
    non_critical_segments = []
    for i in range(len(critical)):
        if i == 0:
            if critical[i] == 0:
                non_critical_segments.append(solution[0:i])
        else:
            if critical[i] - critical[i-1] > 1:
                non_critical_segments.append(solution[critical[i-1]:critical[i]])
    return non_critical_segments


def thick_bresenham_line(p1, p2, grid, thickness=1):
    """Returns True if there is a clear path between p1 and p2 with thickness"""
    x1, y1 = p1
    x2, y2 = p2

    # Generate main Bresenham line
    main_line = np.linspace((x1, y1), (x2, y2), num=max(abs(x2-x1), abs(y2-y1))+1).astype(int)

    # Check nearby points within the thickness range
    for x, y in main_line:
        for dx in range(-thickness, thickness + 1):
            for dy in range(-thickness, thickness + 1):
                if 0 <= x + dx < grid.shape[0] and 0 <= y + dy < grid.shape[1]:
                    if not grid[x + dx, y + dy]:  # Obstacle detected
                        return False
    return True

def bresenham_line(p1, p2, grid):
    """Returns True if there is a clear line of sight between p1 and p2"""
    x1, y1 = p1
    x2, y2 = p2
    points = np.linspace((x1, y1), (x2, y2), num=max(abs(x2-x1), abs(y2-y1))+1).astype(int)
    #print(np.linspace((x1, y1), (x2, y2), num=max(abs(x2-x1), abs(y2-y1))+1).astype(int))
    return all(grid[y, x] for x, y in points)  # Assume 0 is free space, 1 is obstacle

def lazy_theta_smooth(path, grid):
    """Given an existing path, apply Lazy Theta* to remove unnecessary waypoints"""
    if len(path) < 3:
        return path  # Nothing to smooth

    new_path = [path[0]]  # Start with the first waypoint
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1  # Try to connect to the farthest reachable point
        while j > i + 1:
            if thick_bresenham_line(path[i], path[j], grid):  # If direct path is possible
                break  # Shortcut found
            j -= 1
        
        # step through each
        num_steps = j - i
        direction = (np.array(path[j]) - np.array(new_path[-1])) / num_steps
        
        for k in range(i+1,j):
            new_path.append(direction + new_path[-1])
        new_path.append(path[j])  # Add the reachable waypoint
        i = j  # Move index forward

    return new_path

def interpolate_positions(start, end, num_steps):
    """Linearly interpolates positions to match num_steps waypoints"""
    x1, y1 = start
    x2, y2 = end
    interpolated = np.linspace([x1, y1], [x2, y2], num=num_steps).astype(int)
    return [(x, y) for x, y in interpolated]

def lazy_theta_smooth_time_aware(path, grid, thickness=1):
    """Smoothes path while keeping the same number of time steps"""
    if len(path) < 3:
        return path  

    new_path = [path[0]]  
    i = 0
    time_steps = len(path)

    while i < len(path) - 1:
        j = len(path) - 1  

        while j > i + 1:
            if path[j][:2] == path[j - 1][:2]:  
                j -= 1  
                continue

            if thick_bresenham_line(path[i], path[j], grid, thickness):  
                break  
            j -= 1

        new_path.append(path[j])  
        i = j  

    # Ensure new_path has (x, y, t) format
    new_path = [(x, y, t) for t, (x, y) in enumerate(new_path)]  

    # Ensure same number of waypoints by interpolating
    final_path = []
    interp_index = 0

    for t in range(time_steps):
        if interp_index < len(new_path) - 1:
            _, _, next_time = new_path[interp_index + 1]  
            if t >= next_time:
                interp_index += 1  

        if interp_index < len(new_path) - 1:
            start = new_path[interp_index][:2]
            end = new_path[interp_index + 1][:2]
            num_steps = new_path[interp_index + 1][2] - new_path[interp_index][2] + 1
            interpolated_segment = interpolate_positions(start, end, num_steps)
            final_path.append((*interpolated_segment[t - new_path[interp_index][2]], t))
        else:
            final_path.append(new_path[-1])  

    return final_path

from math import sqrt

def path_length(points):
    return sum(sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) for p1, p2 in zip(points, points[1:]))