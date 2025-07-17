import numpy as np
from collections import deque
import random
from typing import List

from pypibt.mapf_utils import get_grid
from pypibt.pibt import CollisionChecker

from pypibt.graph_types.scaled_2d_grid import StaticObstacle, GridMapWithStaticObstacles
import pickle
import os
import hashlib

def _flood_fill_count_clusters(grid: np.ndarray) -> int:
    """
    Counts the number of clusters of 'True' values in a NumPy boolean grid
    using a flood fill (BFS) algorithm. An agent can move up, down, left, or right.

    Args:
        grid (np.ndarray): A 2D NumPy array of boolean values.

    Returns:
        int: The total number of distinct clusters.
    """
    if grid.size == 0:
        return 0

    rows, cols = grid.shape
    visited = np.full((rows, cols), False, dtype=bool)
    colors = np.full((rows, cols), 0, dtype=np.int64)

    cluster_count = 0

    # Define possible movements: (row_offset, col_offset) for up, down, left, right
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    for r in range(rows):
        for c in range(cols):
            # If the current cell is 'True' and has not been visited yet,
            # it's the start of a new cluster.
            if grid[r, c] and not visited[r, c]:
                cluster_count += 1
                # Start a Breadth-First Search (BFS) from this cell to find all
                # connected 'True' cells in the current cluster.
                queue = deque([(r, c)])
                visited[r, c] = True
                colors[r, c] = cluster_count

                while queue:
                    curr_r, curr_c = queue.popleft()

                    # Explore all four possible directions (up, down, left, right)
                    for dr, dc in directions:
                        next_r, next_c = curr_r + dr, curr_c + dc

                        # Check if the next cell is within grid boundaries
                        if 0 <= next_r < rows and 0 <= next_c < cols:
                            # If the next cell is 'True' and hasn't been visited,
                            # add it to the queue and mark it as visited.
                            if grid[next_r, next_c] and not visited[next_r, next_c]:
                                visited[next_r, next_c] = True
                                colors[next_r, next_c] = cluster_count
                                queue.append((next_r, next_c))

    return cluster_count, colors

def find_true_coordinates(grid: np.ndarray) -> list[tuple[int, int]]:
    """
    Takes a 2D NumPy array (grid) and returns the coordinates (row, column)
    of all elements that are True.

    Args:
        grid (np.ndarray): A 2D NumPy array, typically of boolean type.

    Returns:
        list[tuple[int, int]]: A list of (row, column) tuples for all True elements.
                               Returns an empty list if no True elements are found.

    Example:
        >>> # Example 1: Basic usage
        >>> grid1 = np.array([[True, False, True],
        ...                   [False, True, False],
        ...                   [True, True, False]])
        >>> find_true_coordinates(grid1)
        [(0, 0), (0, 2), (1, 1), (2, 0), (2, 1)]

        >>> # Example 2: No True values
        >>> grid2 = np.array([[False, False],
        ...                   [False, False]])
        >>> find_true_coordinates(grid2)
        []

        >>> # Example 3: All True values
        >>> grid3 = np.array([[True, True],
        ...                   [True, True]])
        >>> find_true_coordinates(grid3)
        [(0, 0), (0, 1), (1, 0), (1, 1)]
    """
    if not isinstance(grid, np.ndarray) or grid.ndim != 2:
        print("Error: Input must be a 2D NumPy array.")
        return []

    # Use np.where to find the indices where the grid elements are True
    # np.where returns two arrays: one for row indices, one for column indices
    row_indices, col_indices = np.where(grid)

    # Combine the row and column indices into a list of (row, col) tuples
    coordinates = list(zip(row_indices, col_indices))

    return coordinates

def select_random_start_end_for_grid(grid, n, other_blocked_spots = set()):
    grid_cell_array = grid.to_numpy_array()
    sample_from = [a for a in find_true_coordinates(grid_cell_array) if a not in other_blocked_spots]
    num_clusters, cluster_map = _flood_fill_count_clusters(grid_cell_array)
    starts, ends = [], []
    for _ in range(n):
        x,y = random.choice(sample_from)
        other_blocked_spots.add((x,y))
        starts.append((x, y))
        cluster_to_sample_from = cluster_map[x,y]
        end_locs = cluster_map == cluster_to_sample_from
        end_locations = [a for a in find_true_coordinates(end_locs) if a not in other_blocked_spots]
        if len(end_locations) == 0:
            return starts[:-1], ends
        end_loc = random.choice(end_locations)
        ends.append(end_loc)
        other_blocked_spots.add(end_loc)
    return starts, ends

def coordinates_to_node_id_with_graph_label(grid, coordinates, graph_label):
    return [(int(graph_label), int(grid.get_node_id(*coord))) for coord in coordinates]

def remap_to_current_graph_id(graph_id: int, collision_check: CollisionChecker, blocked_nodes) -> List[int]:
    all_blocked_nodes = []
    for blocked_node in blocked_nodes:
        all_blocked_nodes += [node for graph,node in collision_check.get_other_blocked_nodes(*blocked_node) if graph_id == graph]
    return all_blocked_nodes

def get_row_cols(graph_id: int, collision_check: CollisionChecker, blocked_nodes: List[int]):
    coords = []
    for node in blocked_nodes:
        coords.append(collision_check.graphs[graph_id].row_cols[node])
    return set(coords)

def create_random_problem_inst(collision_check: CollisionChecker, num_per_graph: int):
    """
    Creates a random problem instance given a set of graphs.
    """
    blocked_nodes = []
    result = {}
    for i in range(len(collision_check.graphs)):
        curr_graph_nodes = remap_to_current_graph_id(i, collision_check, blocked_nodes)
        blocked_spots = get_row_cols(i, collision_check, curr_graph_nodes)
        starts, ends = select_random_start_end_for_grid(collision_check.graphs[i], num_per_graph, blocked_spots)
        result[i] ={"start_coord": starts, "end_coord": ends}
        blocked_nodes += coordinates_to_node_id_with_graph_label(collision_check.graphs[i], starts+ends, i)
    return result


def export_problem(collision_check: CollisionChecker, problem, file_path):
    with open(file_path, "w") as f:
        agent_id = 0
        for fleet_id in problem:
            print(fleet_id)
            start = problem[fleet_id]["start_coord"]
            end = problem[fleet_id]["end_coord"]
            assert len(start) == len(end)

            for i in range(len(start)):
                start_node = collision_check.graphs[fleet_id].get_node_id(*start[i])
                end_node = collision_check.graphs[fleet_id].get_node_id(*end[i])
                (sx, sy)= collision_check.graphs[fleet_id].get_node_center(start_node)
                (ex, ey)= collision_check.graphs[fleet_id].get_node_center(end_node)
                f.write(f"{agent_id} {fleet_id} {collision_check.graphs[fleet_id].cell_size} {collision_check.graphs[fleet_id].cell_size} {sx} {sy} {ex} {ey} {collision_check.graphs[fleet_id].num_rows} {collision_check.graphs[fleet_id].num_cols}\n")


def import_problem(file_path, map_file, base_map_scale=10, cache_dir=".hetbench_cache"):

    # Cacheing logic
    sha256_hasher = hashlib.sha256()

    static_obstacles = StaticObstacle(base_map_scale, get_grid(map_file))
    fleets = {}
    signature = []
    with open(file_path) as f:
        for line in f:
            agent, fleet, footprint, vel, start_x, start_y, goal_x, goal_y, width, height = line.split()
            if fleet in fleets:
                fleets[fleet]["agents"].append([int(start_x), int(start_y), int(goal_x), int(goal_y)])
            else:
                fleets[fleet] = {
                    "agents": [[int(start_x), int(start_y), int(goal_x), int(goal_y)]],
                    "footprint": int(footprint),
                    "velocity": int(vel),
                    "width": int(width),
                    "height": int(height)
                }
    for fleet in fleets:
        signature.append(f"{fleets[fleet]['footprint']}-{fleets[fleet]['velocity']}-{fleets[fleet]['width']}-{fleets[fleet]['height']}")

    signature.sort()
    signature = "_".join(signature)

    sha256_hasher.update((signature + ":" + map_file).encode('utf-8'))
    hash = str(sha256_hasher.hexdigest())

    collision_checker = None
    if os.path.exists(os.path.join(cache_dir, hash)):
        print("Cache hit, loading scenario")
        with open(os.path.join(cache_dir, hash), 'rb') as f:
            try:
                coll_check, static_obs = pickle.load(f)
                for graph_id in range(len(coll_check)):
                    problem[graph_id] = {
                        "start_coord": [],
                        "end_coord": []
                    }
                for start_x, start_y, end_x, end_y in fleets[fleet]["agents"]:
                    problem[len(graphs)-1]["start_coord"].append(graphs[-1].from_node_center_to_node_id(start_x, start_y))
                    problem[len(graphs)-1]["end_coord"].append(graphs[-1].from_node_center_to_node_id(end_x, end_y))
                return (coll_check, problem, static_obs)
            except:
                print("Corrupt file overwriting")

    if collision_checker is None:
        graphs = []
        problem = {}
        for fleet in fleets:
            graphs.append(
                GridMapWithStaticObstacles(fleets[fleet]["velocity"], fleets[fleet]["width"], fleets[fleet]["height"],
                                        (0,0), static_obstacles))
            problem[len(graphs)-1] = {
                "start_coord": [],
                "end_coord": []
            }
            for start_x, start_y, end_x, end_y in fleets[fleet]["agents"]:
                problem[len(graphs)-1]["start_coord"].append(graphs[-1].from_node_center_to_node_id(start_x, start_y))
                problem[len(graphs)-1]["end_coord"].append(graphs[-1].from_node_center_to_node_id(end_x, end_y))

        collision_checker = CollisionChecker(graphs)

        with open(os.path.join(cache_dir, hash), 'wb') as f:
            pickle.dump((collision_checker, static_obstacles), f)
        return collision_checker, problem, static_obstacles

