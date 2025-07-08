from pypibt.benchmarks.generate_heterogeneous_problem import import_problem, coordinates_to_node_id_with_graph_label
from pypibt import CollisionChecker, PIBTFromMultiGraph

collision_checker, problem = import_problem("test_scen.0.scen", "assets/room-64-64-8.map")

def solve_problem(problem, collision_check: CollisionChecker):
    graph_space_start = []
    graph_space_end = []
    for graph_index in problem:
        starts = problem[graph_index]['start_coord']
        ends = problem[graph_index]['end_coord']
        graph_space_start += coordinates_to_node_id_with_graph_label(collision_check.graphs[graph_index], starts, graph_index)
        graph_space_end += coordinates_to_node_id_with_graph_label(collision_check.graphs[graph_index], ends, graph_index)
    print(graph_space_start, graph_space_end)
    pibt_solver = PIBTFromMultiGraph(collision_check, graph_space_start, graph_space_end)
    result = pibt_solver.run()
    return graph_space_start, graph_space_end, result

print(solve_problem(problem, collision_checker))