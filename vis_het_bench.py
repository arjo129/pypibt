from pypibt.benchmarks.generate_heterogeneous_problem import import_problem, coordinates_to_node_id_with_graph_label
from pypibt import CollisionChecker, PIBTFromMultiGraph
import numpy as np

collision_checker, problem = import_problem("test_scen.0.scen", "assets/room-64-64-8.map")

def solve_problem(problem, collision_check: CollisionChecker):
    graph_space_start = []
    graph_space_end = []
    for graph_index in problem:
        starts = problem[graph_index]['start_coord']
        ends = problem[graph_index]['end_coord']
        graph_space_start += [(graph_index, start) for start in starts]
        graph_space_end += [(graph_index, end) for end in ends]
    print(graph_space_start, graph_space_end)
    pibt_solver = PIBTFromMultiGraph(collision_check, graph_space_start, graph_space_end)
    result = pibt_solver.run()
    return graph_space_start, graph_space_end, result


def remap_result_to_graph_space(result_vec):
    res = []
    for graph_id, node_id in result_vec:
        res.append(collision_checker.graphs[graph_id].get_node_center(node_id))
    return np.array(res)

starts, ends, solution = solve_problem(problem, collision_checker)
print(f"Time steps: {len(solution)}")
total_length = 0
for time in range(1,len(solution)):
    res = remap_result_to_graph_space(np.array(solution[time])) - remap_result_to_graph_space(np.array(solution[time-1]))
    res = np.linalg.norm(res, axis=1)
    total_length += sum(res)
print(f"Total length: {total_length}")