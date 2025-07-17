from pypibt.benchmarks.generate_heterogeneous_problem import import_problem, coordinates_to_node_id_with_graph_label
from pypibt import CollisionChecker, PIBTFromMultiGraph
import numpy as np
import pygame

collision_checker, problem, static_obs = import_problem("heterogenous_bench/room-64-64-8.1.scen", "assets/room-64-64-8.map")

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


def draw_circle_goal(screen, color, center, radius):
    """
    Draws a circle with a hole (donut shape) at the specified center.

    Args:
        screen: The pygame screen to draw on.
        color: The color of the circle.
        center: The (x, y) coordinates of the circle's center.
    """
    outer_radius = 4 + radius
    inner_radius = radius

    center = np.array(center)
    center *= 10
    center = (center[0], center[1])
    # Draw the outer circle
    pygame.draw.circle(screen, color, center, outer_radius)

    # Draw the inner circle with the background color to create the hole
    background_color = (0, 0, 0)  # Assuming black background
    pygame.draw.circle(screen, background_color, center, inner_radius)

def visualize_solution(graphs, starts, ends, obstacles, result, sizes= [5,8,6]):
    pygame.init()
    print(starts, ends)
    # Set up the window
    width, height = 800, 800
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Heterogenous PiBT visualization")

    print(f"starts: {starts}")
    print(f"ends: {ends}")


    goal_center = [graphs[starts[i][0]].get_node_center(ends[i][1]) for i in range(len(starts))]
    goal_center[0] *= 10
    goal_center[1] *= 10
    pygame.display.flip()
    running = True
    i = 0


    blue = (135, 206, 250)  # Light blue
    red = (255, 0, 0)
    purple =(0,255,255)
    colors = [blue, red, purple] # Add more colors
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        #screen.fill((0, 0, 0))  # Clear the screen with a black background
        if i > 0:
            prev_results = result[(i - 1) % len(result)]
            curr_results = result[i % len(result)]

            steps = 5  # Number of interpolation steps
            for step in range(steps):
                screen.fill((0, 0, 0))  # Clear the screen with a black background
                #obstacles.visualize(screen, (255,255,255))
                #for index, graph in enumerate(graphs):
                #    graph.visualize(screen, colors[index])
                for agent_id, (curr_graph_id,_) in enumerate(curr_results):
                    draw_circle_goal(screen, colors[curr_graph_id], goal_center[agent_id], sizes[curr_graph_id])
                for agent_id, (prev_loc, curr_loc) in enumerate(zip(prev_results, curr_results)):
                    prev_graph_id, prev_node_id = prev_loc
                    curr_graph_id, curr_node_id = curr_loc

                    prev_center = graphs[prev_graph_id].get_node_center(prev_node_id)
                    curr_center = graphs[curr_graph_id].get_node_center(curr_node_id)

                    interpolated_center = (
                    prev_center[0] + (curr_center[0] - prev_center[0]) * (step / steps),
                    prev_center[1] + (curr_center[1] - prev_center[1]) * (step / steps),
                    )

                    interpolated_center = np.array(interpolated_center)
                    interpolated_center *= 10
                    interpolated_center = (interpolated_center[0], interpolated_center[1])

                    pygame.draw.circle(screen, colors[curr_graph_id], interpolated_center, sizes[curr_graph_id])
                #pygame.image.save(screen, f"screenshot_{i*steps+step}.png")
                pygame.display.flip()
                #pygame.time.delay(50)  # Delay for smooth interpolation

        i += 1
        if i >= len(result):
            break

    pygame.quit()

def remap_result_to_graph_space(result_vec):
    res = []
    for graph_id, node_id in result_vec:
        res.append(collision_checker.graphs[graph_id].get_node_center(node_id))

    return np.array(res)

starts, ends, solution = solve_problem(problem, collision_checker)
visualize_solution(collision_checker.graphs, starts, ends, static_obs, solution, [graph.cell_size for graph in collision_checker.graphs])
print(f"Time steps: {len(solution)}")
total_length = 0
for time in range(1,len(solution)):
    res = remap_result_to_graph_space(np.array(solution[time])) - remap_result_to_graph_space(np.array(solution[time-1]))
    res = np.linalg.norm(res, axis=1)
    total_length += sum(res)
print(f"Total length: {total_length}")