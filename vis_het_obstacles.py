from pypibt import CollisionChecker, PIBTFromMultiGraph
from pypibt.graph_types.scaled_2d_grid import GridMapWithStaticObstacles, StaticObstacle
from pypibt.mapf_utils import get_grid
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


def visualize_solution(graphs, starts, ends, obstacles, result, sizes= [10,8]):
    pygame.init()
    print(starts, ends)
    # Set up the window
    width, height = 500, 500
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Heterogenous PiBT visualization")

    print(result)


    goal_center = [graphs[starts[i][0]].get_node_center(ends[i][1]) for i in range(len(starts))]
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
                pygame.time.delay(50)  # Delay for smooth interpolation

        i += 1
        if i >= len(result):
            break

    pygame.quit()

def label_by_graph(start_ends, graph_id):
    labelled_starts = []
    unlabelled_ends = []
    for start, end in start_ends:
        labelled_starts.append((graph_id, start))
        unlabelled_ends.append((graph_id, end))
    return labelled_starts,unlabelled_ends

obstacles = StaticObstacle(50, get_grid("assets/room-64-64-8.map"))
graph1 = GridMapWithStaticObstacles(50, 8, 8, (0, 0), obstacles)
graph2 = GridMapWithStaticObstacles(35, 16, 5, (0, 0), obstacles)

start_ends, constraints = graph1.select_random_start_end(3)
print(start_ends)
starts,ends = label_by_graph(start_ends, 0)

collision_check = CollisionChecker([graph1,  graph2])

pibt_solver = PIBTFromMultiGraph(collision_check, starts, ends)
result = pibt_solver.run()
blue = (135, 206, 250)  # Light blue
red = (255, 0, 0)

visualize_solution([graph1, graph2], starts, ends, obstacles, result)
