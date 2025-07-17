from pypibt import CollisionChecker, PIBTFromMultiGraph
from pypibt.graph_types.scaled_2d_grid import GridMapWithStaticObstacles, StaticObstacle
from pypibt.benchmarks.generate_heterogeneous_problem import *
from pypibt.mapf_utils import get_grid
import pygame
import time

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

    # Draw the outer circle
    pygame.draw.circle(screen, color, center, outer_radius)

    # Draw the inner circle with the background color to create the hole
    background_color = (0, 0, 0)  # Assuming black background
    pygame.draw.circle(screen, background_color, center, inner_radius)

def draw_triangle_agent(screen, color, center, rotation, size):
    """
    Draws a triangle representing an agent, where the tip of the triangle indicates the direction of the front.

    Args:
        screen: The pygame screen to draw on.
        color: The color of the triangle.
        center: The (x, y) coordinates of the triangle's center.
        rotation: The rotation angle in degrees (0 points up, 90 points right, etc.).
        size: The size of the triangle (distance from center to tip).
    """
    # Calculate the vertices of the triangle
    tip = (center[0] + size * pygame.math.cos(pygame.math.radians(rotation)),
           center[1] - size * pygame.math.sin(pygame.math.radians(rotation)))
    left = (center[0] + size * pygame.math.cos(pygame.math.radians(rotation + 120)),
            center[1] - size * pygame.math.sin(pygame.math.radians(rotation + 120)))
    right = (center[0] + size * pygame.math.cos(pygame.math.radians(rotation - 120)),
             center[1] - size * pygame.math.sin(pygame.math.radians(rotation - 120)))

    # Draw the triangle
    pygame.draw.polygon(screen, color, [tip, left, right])


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
            
            steps = 10  # Number of interpolation steps
            for step in range(steps):
                screen.fill((0, 0, 0))  # Clear the screen with a black background
                obstacles.visualize(screen, (255,255,255))
                for index, graph in enumerate(graphs):
                    graph.visualize(screen, colors[index])
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

                    pygame.draw.circle(screen, colors[curr_graph_id], interpolated_center, sizes[curr_graph_id])
                pygame.display.flip()
                pygame.time.delay(10)  # Delay for smooth interpolation

        i += 1
        if i >= len(result):
            break

    pygame.quit()

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

def label_by_graph(start_ends, graph_id):
    labelled_starts = []
    unlabelled_ends = []
    for start, end in start_ends:
        labelled_starts.append((graph_id, start))
        unlabelled_ends.append((graph_id, end))
    return labelled_starts,unlabelled_ends


print("Creating graph")

obstacles = StaticObstacle(10, get_grid("assets/room-64-64-8.map"))

n_graphs =2

fixed_width= 60
fixed_height = 60
initial_start_x = 0
initial_start_y = 0
initial_size = 1

# Create the GridMap instance
current_start_pos = (10,10)
graph1 = GridMapWithStaticObstacles(1, int(fixed_width/1), int(fixed_height/1), current_start_pos, obstacles)

with open('delete_me.csv', 'w', newline='\n') as file:

    for B_factor in range(0,35,2):
        computation = [graph1]
        current_size = initial_size * (1+B_factor*0.5)
        # Create the GridMap instance
        graph = GridMapWithStaticObstacles(current_size, int(fixed_width/current_size), int(fixed_height/current_size), current_start_pos, obstacles)
        computation.append(graph)
        print(f"Generated GridMap: size={current_size}, start_pos={current_start_pos}")
        for num_fleets in range(2, len(computation)+1):
            print("Precomputing collisions")
            collision_check = CollisionChecker(computation[:num_fleets])

            k = 10
            for i in range(10):
                try:
                    problem = create_random_problem_inst(collision_check, k)
                    print("Problem")
                    print(problem)

                    start_time = time.time()
                    starts, ends, result = solve_problem(problem, collision_check)
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    file.write(f"{num_fleets}, {k}, {k*num_fleets}, {(1+B_factor*0.5)}, {elapsed_time}\n")
                    print(f"{num_fleets}, {k}, {k*num_fleets}, {(1+B_factor*0.5)}, {elapsed_time}\n")
                except:
                    print("Error")
                    continue

