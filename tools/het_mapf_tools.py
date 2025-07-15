import yaml
from yaml import CLoader as Loader, CDumper as Dumper
import json
from collections import defaultdict
from pypibt.mapf_utils import get_grid
from pypibt.benchmarks.generate_heterogeneous_problem import import_problem
import argparse
import tqdm

def ascii_map_to_occupancy(ascii_map: str):
    grid = get_grid(ascii_map)
    obstacle_coords = [(x, y) for y, row in enumerate(grid) for x, val in enumerate(row) if val == False]
    occupancy = {}

    """
    for ox, oy in tqdm.tqdm(obstacle_coords):
        # For each 10x10 block, mark all its constituent 1x1 cells as occupied
        for y_block in range(oy * 10, (oy + 1) * 10):
            if y_block not in occupancy:
                occupancy[y_block] = []
            for x_block in range(ox * 10, (ox + 1) * 10):
                occupancy[y_block].append(x_block)
    """
    for ox, oy in tqdm.tqdm(obstacle_coords):
        # Calculate the center of the 10x10 block in terms of 1x1 cells
        center_x = ox *10  # 5 is half of the block size (10)
        center_y = oy *10

        # For each 10x10 block, mark all its constituent 1x1 cells as occupied,
        # scaling from the center
        for y_block in range(center_y, center_y + 10):
            if y_block not in occupancy:
                occupancy[y_block] = []
            for x_block in range(center_x, center_x + 10):
                occupancy[y_block].append(x_block)

    """
    max_y, max_x = grid.shape
    for y in range(-1, max_y*10):
        if y not in occupancy:
            occupancy[y] = []
        occupancy[y].append(-1)
        occupancy[y].append(max_x*10)

    for x in range(-1, max_x*10):
        if -1 not in occupancy:
            occupancy[-1] = []
        if max_y*10 not in occupancy:
            occupancy[max_y*10] = []
        occupancy[-1].append(x)
        occupancy[max_y*10].append(x)
    """
    return occupancy

def load_agents(map_file, agent_file_path: str, num_agents=2):
    agents = {}

    collision_checker, problem, _ = import_problem(agent_file_path, map_file)


    for graph_index in problem:
        starts = problem[graph_index]['start_coord']
        ends = problem[graph_index]['end_coord']
        for i in range(min(len(starts), num_agents)):
            name = f"A{graph_index}_{i}"  # A, B, C, ...
            start_coord = collision_checker.graphs[graph_index].get_node_center(starts[i])
            end_coord = collision_checker.graphs[graph_index].get_node_center(ends[i])
            agent = {
                "start": [ int(round(start_coord[0] )),
                          int(round(start_coord[1]))],
                "goal": [ int(round(end_coord[0])),
                         int(round(end_coord[1]))],
                "yaw": 0.0,
                "radius": collision_checker.graphs[graph_index].cell_size/2,
                "speed": collision_checker.graphs[graph_index].cell_size,
                "spin": 5000
            }
            agents[name] = agent

    return agents


def build_yaml(ascii_map_str, agent_file_path, num_agents=15):

    occupancy = ascii_map_to_occupancy(ascii_map_str)
    agents = load_agents(ascii_map_str, agent_file_path)

    # Default bounding box and cell size
    output = {
        "agents": agents,
        "obstacles": [],
        "occupancy": occupancy,
        "cell_size": 1,
        "camera_bounds": [
            [-1.0, -1.0],
            [64+ 1.0, 64 + 1.0]
        ]
    }

    with open("test_new_status.yaml", "w") as f:
        yaml.dump(output, f)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate YAML from MAPF map and scenario files.")
    parser.add_argument("--map_file", type=str, required=True, help="Path to the map file.")
    parser.add_argument("--scene_file", type=str, required=True, help="Path to the scenario file.")
    parser.add_argument("--out_file", type=str)
    #parser.add_argument("--num_agents", type=int, default=15, help="Number of agents to include.")
    args = parser.parse_args()

    yaml_str = build_yaml(args.map_file, args.scene_file)