import argparse
from pypibt import (
    PIBT,
    get_grid,
    get_scenario,
    is_valid_mapf_solution,
    save_configs_for_visualizer,
    cost_of_solution
)
from pypibt.mapf_utils import to_per_agent_trajectory
from pypibt.smoothing import smoothe_all_paths
import time

def create_parser():
    """Creates an argparse parser for command-line arguments."""
    parser = argparse.ArgumentParser(description="Run a multi-agent simulation.")

    parser.add_argument(
        "--map",
        type=str,
        required=True,
        help="Path to the map file.",
    )

    parser.add_argument(
        "--scenario",
        type=str,
        required=True,
        help="Name of the scenario to run.",
    )

    parser.add_argument(
        "--agents",
        type=int,
        required=True,
        help="Number of agents in the simulation.",
    )

    return parser

if __name__ == "__main__":
    parser = create_parser()
    args = parser.parse_args()

    print(f"Map file: {args.map}")
    print(f"Scenario: {args.scenario}")
    print(f"Number of agents: {args.agents}")
    grid = get_grid(args.map)
    starts, goals = get_scenario(args.scenario, int(args.agents))

    # solve MAPF
    pibt = PIBT(grid, starts, goals, 42)
    t0 = time.perf_counter()
    plan = pibt.run(max_timestep=1000)
    t1 = time.perf_counter()
    print(f"solved: {is_valid_mapf_solution(grid, starts, goals, plan)}")
    per_agent_traj = to_per_agent_trajectory(plan)
    smoothe_all_paths(per_agent_traj, grid)
    t2 = time.perf_counter()
    print(f"PiBT solved in {t1-t0:.4f} seconds")
    print(f"Smoothing found in {t2-t0:.4f} seconds")
