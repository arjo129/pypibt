import pygame
import numpy as np
from pypibt import get_grid, path_length
from pypibt.smoothing import smoothe_all_paths
from pypibt.mapf_utils import scale_paths
import re
import random
import sys

def _is_straight_line_clear(grid, start, end, critical_section):
    """
    Checks if a straight line between two points is clear of obstacles.

    Args:
        grid (np.ndarray): 2D numpy array representing the grid.
        start (tuple): (x, y) tuple representing the start point.
        end (tuple): (x, y) tuple representing the end point.

    Returns:
        bool: True if the line is clear, False otherwise.
    """
    x1, y1 = start
    x2, y2 = end

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if x1 < 0 or x1 >= grid.shape[1] or y1 < 0 or y1 >= grid.shape[0] or not grid[y1, x1] or critical_section[y1, x1]:
            return False

        if x1 == x2 and y1 == y2:
            return True

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy


def straight_path(grid, path, critical_sections):
    """
    Strighten the path without running into obstacles. Essentially, reduce the number of jagged edges by
    checking if a straight line between two points is clear of obstacles.

    A.k.a "String pulling"
    """
    new_path = [(path[0][0], path[0][1], 0)]
    i = 0
    while i < len(path) - 1:
        j = i + 1
        critcal_section_check = critical_sections[i]
        while j < len(path) - 1 and _is_straight_line_clear(grid, path[i][:2], path[j][:2], critcal_section_check) and path[j][:2] != path[j-1][:2] and critical_sections[j][path[j][1], path[j][0]] == False:
            j += 1
            critcal_section_check = critical_sections[j] # The final velocity might be lower than the max velocity, so we need to check the critical sections of the entire path

        prev_pos = np.array(new_path[-1][:2]).astype(np.float64)
        print(prev_pos)
        cur_pos = np.array(path[j][:2]).astype(np.float64)
        diff = cur_pos - prev_pos
        length = np.linalg.norm(diff)
        step = diff / (j-i)
        if length > 0:
            for k in range(i, j):
                prev_pos += step
                new_path.append((prev_pos[0], prev_pos[1]))
        else:
            new_path.append((path[j][0], path[j][1], j))
        i = j
        
    return new_path


def identify_all_critical_regions(paths, grid):
    """
    Identifies critical regions in a path based on the density of agents

    Args:
        path (dict): dict of list of (x, y) tuples representing the path.
        grid (numpy.ndarray): 2D NumPy array representing obstacles (False for occupied).

    Returns:
        list: List of (numpy.ndarray) True if a cell is critical at a certain time, False otherwise.
    """
    critical_regions = []
    if len(paths) < 1:
        return critical_regions
    for t in range(len(paths[0])):
        critical_zone = np.full(grid.shape, False)

        agent_unsafe_zones = []
        for agent in paths:
            x1, y1 = paths[agent][t]
            x1 = int(x1)
            y1 = int(y1)
            to_mark1 = []
            for i in range(-1,2):
                for j in range(-1,2):
                    if x1+i >= 0 and x1+i < grid.shape[1] and y1+j >= 0 and y1+j < grid.shape[0] and grid[y1+j, x1+i]:
                        to_mark1.append((x1+i, y1+j))
            agent_unsafe_zones.append(set(to_mark1))
        print(len(agent_unsafe_zones))
        
        for i in range(len(agent_unsafe_zones)):
            for j in range(i+1, len(agent_unsafe_zones)):
                if agent_unsafe_zones[i].intersection(agent_unsafe_zones[j]):
                    for coord in agent_unsafe_zones[i].union(agent_unsafe_zones[j]):
                        critical_zone[coord[1], coord[0]] = True
        critical_regions.append(critical_zone)
    return critical_regions
                    

def visualize_agent_motion_with_obstacles(filepath, obstacles, grid_scale=4, screen_width=1024, screen_height=1024):
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
                x = int(float(x_str)) #* grid_scale
                y = int(float(y_str)) #* grid_scale
                coords.append((x, y))

            for i in range(len(coords)):
                if i not in agent_paths:
                    agent_paths[i] = []
                agent_paths[i].append(coords[i])

    path_bef = 0
    path_aft = 0
    og_agent_paths = agent_paths.copy()
    agent_paths = smoothe_all_paths(agent_paths, obstacles)
    agent_paths = scale_paths(agent_paths, grid_scale)
    """
    normal_paths = {}
    for agent in agent_paths:
        new_path = [(int(x[0]/grid_scale), int(x[1]/grid_scale)) for x in agent_paths[agent]]
        normal_paths[agent] = new_path
    critical_sections = identify_all_critical_regions(normal_paths, obstacles)
    
    for agent in agent_paths:
        new_path = [(int(x[0]/grid_scale), int(x[1]/grid_scale)) for x in agent_paths[agent]]
        path_bef += path_length(new_path)
        normalized = straight_path(obstacles, new_path, critical_sections)
        path_aft += path_length(normalized)
        normalized = [(x[0] * grid_scale, x[1] * grid_scale) for x in normalized]
        agent_paths[agent] = normalized
    """
    #print(og_agent_paths[7])
    #print(agent_paths[7])

    #print("Path length before smoothing: ", path_bef)
    #print("Path length after smoothing: ", path_aft)
    #print("Path length reduction: ", path_bef - path_aft)
    #print("Path length reduction percentage: ", (path_bef - path_aft) / path_bef * 100, "%")

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
    print("Number of agents: ", num_agents)
    num_collisions = 0
    collision_location = []
    involved_agents = []
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
                        if agent_id in involved_agents:
                            pygame.draw.lines(screen, (255, 0, 0), False, lines)
                        else:
                            pygame.draw.lines(screen, (0, 0, 255), False, lines)

                    pygame.draw.circle(screen, colors[agent_id], (agent_x + grid_scale/2, agent_y + grid_scale/2), 5)
                    """
                    for other_agent_id, other_path in agent_paths.items():
                        if agent_id != other_agent_id and current_timestep < len(other_path):
                            other_x, other_y = other_path[current_timestep]
                            distance = np.sqrt((agent_x - other_x) ** 2 + (agent_y - other_y) ** 2)
                            if distance < 1:
                                print(f"Distance between Agent {agent_id} and Agent {other_agent_id} at timestep {current_timestep}: {distance}")
                                print(f"Warning: Agent {agent_id} is within 1 unit of Agent {other_agent_id} at timestep {current_timestep}")
                                # Draw a green square at the collision location
                                collision_x = int((agent_x + other_x) / 2 // grid_scale) * grid_scale
                                collision_y = int((agent_y + other_y) / 2 // grid_scale) * grid_scale
                                collision_location.append((collision_x, collision_y))
                                involved_agents.append(agent_id)
                                involved_agents.append(other_agent_id)
                                num_collisions += 1
                    
                    
                    mouse_pos = pygame.mouse.get_pos()
                    if pygame.mouse.get_pressed()[0]:  # Check if left mouse button is pressed
                        if agent_circle.collidepoint(mouse_pos):
                            print(f"Agent ID: {agent_id}")
                    """
            current_timestep +=1
            if current_timestep >= len(agent_paths[0]):
                current_timestep = 0
                print("Found ", num_collisions, " collisions")
                num_collisions = 0
                raise Exception("Done")

        pygame.display.flip()
        pygame.time.delay(10)

    pygame.quit()

# Example usage:
filepath = "output.txt"

# Create sample obstacle NumPy array
#obstacle_array = get_grid("assets/room-64-64-8.map")
#obstacle_array = get_grid("assets/Berlin_1_256.map")
#obstacle_array = get_grid("assets/random-32-32-10.map")
obstacle_array = get_grid(sys.argv[1])
visualize_agent_motion_with_obstacles(filepath, obstacle_array)