import pygame
import numpy as np
from pypibt import get_grid, lazy_theta_smooth_time_aware, path_length
import re
import random

def visualize_agent_motion_with_obstacles(filepath, obstacles, grid_scale=15, screen_width=1000, screen_height=1000):
    """
    Visualizes agent motion with obstacles represented by a NumPy array.

    Args:
        filepath (str): Path to the file containing agent coordinates.
        obstacles (numpy.ndarray): 2D NumPy array representing obstacles (True for occupied).
        grid_scale (int): Scaling factor for the coordinates to fit on the screen.
        screen_width (int): Width of the Pygame screen.
        screen_height (int): Height of the Pygame screen.
    """

    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Agent Motion with Obstacles")

    try:
        with open(filepath, 'r') as file:
            lines = file.readlines()
    except FileNotFoundError:
        print(f"Error: Agent data file '{filepath}' not found.")
        return

    agent_paths = {}
    for line in lines:
        match = re.match(r'(\d+):\s*(.*)', line)
        if match:
            coords = []
            coord_pairs = re.findall(r'\(([\d\.-]+),([\d\.-]+)\)', match.group(2))
            for x_str, y_str in coord_pairs:
                x = float(x_str) * grid_scale
                y = float(y_str) * grid_scale
                coords.append((x, y))

            for i in range(len(coords)):
                if i not in agent_paths:
                    agent_paths[i] = []
                agent_paths[i].append(coords[i])

    path_bef = 0
    path_aft = 0
    for agent in agent_paths:
        
        new_path = [(int(x[0]/grid_scale), int(x[1]/grid_scale)) for x in agent_paths[agent]]
        path_bef += path_length(new_path)
        normalized = lazy_theta_smooth_time_aware(new_path, obstacles)
        path_aft += path_length(normalized)
        normalized = [(x[0] * grid_scale, x[1] * grid_scale) for x in normalized]
        agent_paths[agent] = normalized
    print("Path length before smoothing: ", path_bef)
    print("Path length after smoothing: ", path_aft)
    print("Path length reduction: ", path_bef - path_aft)
    print("Path length reduction percentage: ", (path_bef - path_aft) / path_bef * 100, "%")
    num_agents = len(agent_paths)
    # Define a list of colors for the agents
    colors = [
        (255, 0, 0),    # Red
        (0, 255, 0),    # Green
        (0, 0, 255),    # Blue
        (255, 255, 0),  # Yellow
        (255, 0, 255),  # Magenta
        (0, 255, 255),  # Cyan
        (128, 0, 0),    # Maroon
        (0, 128, 0),    # Dark Green
        (0, 0, 128),    # Navy
        (128, 128, 0),  # Olive
        (128, 0, 128),  # Purple
        (0, 128, 128),  # Teal
        (192, 192, 192) # Silver
    ]

    # Ensure there are enough colors for all agents
    if num_agents > len(colors):
        random.seed(42)
        while len(colors) < num_agents:
            colors.append((random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)))
    running = True
    current_timestep = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))

        # Draw obstacles from NumPy array
        rows, cols = obstacles.shape
        for row in range(rows):
            for col in range(cols):
                if obstacles[row, col]:
                    x = col * grid_scale 
                    y = row * grid_scale 
                    pygame.draw.rect(screen, (0, 0, 0), (x, y, grid_scale, grid_scale))

        
        if current_timestep in range(len(agent_paths[0])):
            for agent_id, path in agent_paths.items():
                if current_timestep < len(path):
                    agent_x, agent_y = path[current_timestep]
                    if current_timestep > 0:
                        lines = [(px + grid_scale / 2, py + grid_scale / 2) for px, py in path[:current_timestep+1]]
                        pygame.draw.lines(screen, (0, 0, 255), False, lines)
                    pygame.draw.circle(screen, colors[agent_id], (agent_x + grid_scale/2, agent_y + grid_scale/2), 5)

            current_timestep +=1
            if current_timestep >= len(agent_paths[0]):
                current_timestep = 0

        pygame.display.flip()
        pygame.time.delay(100)

    pygame.quit()

# Example usage:
filepath = "output.txt"

# Create sample obstacle NumPy array
obstacle_array = get_grid("assets/room-64-64-8.map")

visualize_agent_motion_with_obstacles(filepath, obstacle_array)