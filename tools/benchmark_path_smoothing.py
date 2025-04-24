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
import math

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



def count_collisions_at_end_implies_final(trajectory_list):
    """
    Counts collisions between agents, ignoring collisions where one agent
    is at its final location (the last point in its trajectory).

    Args:
        trajectory_list (dict): A dictionary where keys are agent IDs and
                                 values are lists of (x, y) coordinate tuples
                                 representing their trajectories. The last point
                                 in each trajectory is assumed to be the final location.

    Returns:
        int: The total number of collisions detected (excluding those with
             agents at their final locations).
    """
    agents = list(trajectory_list.keys())
    num_agents = len(agents)
    collision_count = 0

    def distance(p1, p2):
        """Calculates the Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    # Iterate through all unique pairs of agents
    for i in range(num_agents):
        for j in range(i + 1, num_agents):
            agent1_id = agents[i]
            agent2_id = agents[j]
            trajectory1 = trajectory_list[agent1_id]
            trajectory2 = trajectory_list[agent2_id]
            end_loc1 = trajectory1[-1] if trajectory1 else None
            end_loc2 = trajectory2[-1] if trajectory2 else None

            # Check for collision at each corresponding time step
            min_len = min(len(trajectory1), len(trajectory2))
            for k in range(min_len):
                pos1 = trajectory1[k]
                pos2 = trajectory2[k]

                # Check if either agent is at their final location
                agent1_at_end = (end_loc1 is not None and pos1[0] == end_loc1[0] and pos1[1] == end_loc1[1])
                agent2_at_end = (end_loc2 is not None and pos2[0] == end_loc2[0] and pos2[1] == end_loc2[1])

                if not (agent1_at_end and agent2_at_end) and distance(pos1, pos2) < 1:
                    collision_count += 1
                    break  # Count each pair's collision only once

    return collision_count


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
    paths = smoothe_all_paths(per_agent_traj, grid)
    t2 = time.perf_counter()
    print(f"PiBT solved in {t1-t0:.4f} seconds")
    print(f"Smoothing found in {t2-t0:.4f} seconds")
    print(f"Collisions introduced: {count_collisions_at_end_implies_final(paths)}")
