from pypibt import CollisionChecker, PIBTFromMultiGraph
from pypibt.graph_types.scaled_2d_grid import GridMap
import pygame


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

def get_pibt_goals(collision_checker: CollisionChecker, start_coords, end_coords):
    start_configs = []
    end_configs = []
    for i in range(len(start_coords)):
        graph_id = start_coords[i][2]
        g = collision_checker.graphs[graph_id]
        start_node_id = g.get_node_id(*start_coords[i][:2])
        end_node_id = g.get_node_id(*end_coords[i][:2])
        start_configs.append((graph_id, start_node_id))
        end_configs.append((graph_id, end_node_id))
    return start_configs, end_configs

def visualize_solution(graphs, starts, ends, result, sizes= [10,8]):
    pygame.init()

    # Set up the window
    width, height = 500, 500
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Heterogenous PiBT visualization")


    goals = [graphs[starts[i][2]].get_node_id(*ends[i]) for i in range(len(starts))]
    goal_center = [graphs[starts[i][2]].get_node_center(goals[i]) for i in range(len(starts))]
    pygame.display.flip()
    running = True
    i = 0


    blue = (135, 206, 250)  # Light blue
    red = (255, 0, 0)
    colors = [blue, red] # Add more colors
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
                for index, graph in enumerate(graphs):
                    graph.visualize(screen, colors[index])
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
                    draw_circle_goal(screen, colors[curr_graph_id], goal_center[agent_id], sizes[curr_graph_id])                pygame.display.flip()
                pygame.display.flip()
                pygame.time.delay(50)  # Delay for smooth interpolation

        i += 1
        if i >= len(result):
            break

    pygame.quit()

trajectory = [(0,4,0), (1,7)]
trajectory2 = [(0,4,1), (1,3)]
trajectory3 = [(0,2,1), (1,4)]

graph1 = GridMap(50, 8, 8, (50, 50))
graph2 = GridMap(25, 16, 5, (252, 52))

collision_check = CollisionChecker([graph1, graph2])
starts, ends = get_pibt_goals(collision_check, (trajectory[0], trajectory2[0], trajectory3[0]), (trajectory[-1], trajectory2[-1], trajectory3[-1]))
#starts, ends = get_pibt_goals(collision_check, (trajectory[0], trajectory2[0]), (trajectory[-1], trajectory2[-1]))

pibt_solver = PIBTFromMultiGraph(collision_check, starts, ends)
result = pibt_solver.run()
blue = (135, 206, 250)  # Light blue
red = (255, 0, 0)

visualize_solution([graph1, graph2], (trajectory[0], trajectory2[0], trajectory3[0]), (trajectory[-1], trajectory2[-1], trajectory3[-1]), result)
