import yaml
from collections import defaultdict
from pypibt.mapf_utils import get_grid, get_scenario
import argparse

# Adjustable agent parameters
DEFAULT_AGENT = {
    "yaw": 0.0,
    "radius": 0.5,
    "speed": 0.75,
    "spin": 1.0471975511965976
}

def ascii_map_to_occupancy(ascii_map: str):
    grid = get_grid(ascii_map)
    obstacle_coords = [(x, y) for y, row in enumerate(grid) for x, val in enumerate(row) if val == False]
    occupancy = {}
    for x, y in obstacle_coords:
        if y not in occupancy:
            occupancy[y] = [x]
        else:
            occupancy[y].append(x)

    max_y, max_x = grid.shape
    for y in range(-1, max_y):
        if y not in occupancy:
            occupancy[y] = []
        occupancy[y].append(-1)
        occupancy[y].append(max_x)

    for x in range(-1, max_x):
        if -1 not in occupancy:
            occupancy[-1] = []
        if max_y not in occupancy:
            occupancy[max_x] = []
        occupancy[-1].append(x)
        occupancy[max_y].append(x)
    return occupancy

def load_agents(agent_file_path: str, num_agents: int):
    agents = {}

    with open(agent_file_path, 'r') as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        if line.startswith("version"):
            continue
        parts = line.strip().split()
        if len(parts) < 9:
            continue

        if len(agents) >= num_agents:
            break
        _, _, _, _, sx, sy, gx, gy, _ = parts
        name = f"A{i}"  # A, B, C, ...
        agent = {
            "start": [int(sx), int(sy)],
            "goal": [int(gx), int(gy)],
            **DEFAULT_AGENT
        }
        agents[name] = agent

    return agents


def build_yaml(ascii_map_str, agent_file_path, num_agents=15):
    lines = ascii_map_str.strip().split("\n")
    map_height = len(lines)

    occupancy = ascii_map_to_occupancy(ascii_map_str)
    agents = load_agents(agent_file_path, num_agents=num_agents)

    # Default bounding box and cell size
    output = {
        "agents": agents,
        "obstacles": [],
        "occupancy": occupancy,
        "cell_size": 1.0,
        "camera_bounds": [
            [-1.0, -1.0],
            [len(lines[0]) + 1.0, map_height + 1.0]
        ]
    }

    return yaml.dump(output, sort_keys=False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate YAML from MAPF map and scenario files.")
    parser.add_argument("--map_file", type=str, required=True, help="Path to the map file.")
    parser.add_argument("--scene_file", type=str, required=True, help="Path to the scenario file.")
    parser.add_argument("--num_agents", type=int, default=15, help="Number of agents to include.")
    args = parser.parse_args()

    yaml_str = build_yaml(args.map_file, args.scene_file, num_agents=args.num_agents)
    print(yaml_str)