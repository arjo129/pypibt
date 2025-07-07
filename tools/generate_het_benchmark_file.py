from pypibt.benchmarks.generate_heterogeneous_problem import create_random_problem_inst, export_problem
from pypibt.graph_types.scaled_2d_grid import GridMapWithStaticObstacles, StaticObstacle
from pypibt.pibt import CollisionChecker
from pypibt.mapf_utils import get_grid
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process map and fleet/agent configurations.")

    parser.add_argument('--map_file', type=str, required=True,
                        help='Path to the map file.')
    parser.add_argument('--scenarios', type=int, required=True,
                        help='Number of scenarios to export')
    parser.add_argument('--outfile', type=str, required=True,
                        help='Output file')
    parser.add_argument('--num_fleets', type=int, required=True,
                        help='Number of fleets.')
    parser.add_argument('--num_agents', type=int, required=True,
                        help='Number of agents.')
    parser.add_argument('--b_factor', type=int, required=True,
                        help='B-factor value.')
    parser.add_argument('--width', type=int, required=True,
                        help='Width')
    parser.add_argument('--height', type=int, required=True,
                        help='Height')

    args = parser.parse_args()

    print(f"Map File: {args.map_file}")
    print(f"Number of Fleets: {args.num_fleets}")
    print(f"Number of Agents: {args.num_agents}")
    print(f"B-Factor: {args.b_factor}")
    obstacles = StaticObstacle(10, get_grid(args.map_file))
    graphs = []
    cell_size =1
    for i in range(args.num_fleets):
        graph = GridMapWithStaticObstacles(cell_size, int(args.width / cell_size), int(args.height / cell_size), (0,0), obstacles)
        graphs.append(graph)
        cell_size += args.b_factor

    collision_check = CollisionChecker(graphs)
    for s in range(args.scenarios):
        problem = create_random_problem_inst(collision_check, args.num_fleets)
        export_problem(collision_check, problem, f"{args.outfile}.{s}.scen")