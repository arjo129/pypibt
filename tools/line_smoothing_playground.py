from pypibt import get_grid
import numpy as np
import pygame

t1 = [(247.0, 170.0), (246.0, 171.0), (245.0, 171.0), (244.0, 172.0), (243.0, 172.0), (242.0, 172.0), (241.0, 171.0), (240.0, 170.0), (239.0, 171.0), (238.0, 172.0), (237.0, 171.0), (236.0, 170.0), (235.0, 170.0), (234.0, 171.0), (233.0, 170.0), (232.0, 171.0), (231.0, 170.0), (230.0, 169.0), (229.0, 168.0), (228.0, 167.0), (227.0, 166.0), (226.0, 165.0), (225.0, 164.0), (224.0, 163.0), (223.0, 162.0), (222.0, 161.0), (221.0, 160.0), (220.0, 159.0), (221.0, 158.0), (220.0, 157.0), (219.0, 156.0), (218.0, 155.0), (217.0, 154.0), (218.0, 153.0), (217.0, 152.0), (216.0, 151.0), (216.0, 150.0), (217.0, 149.0), (216.0, 148.0), (215.0, 147.0), (216.0, 146.0), (217.0, 145.0), (217.0, 144.0), (217.0, 143.0), (217.0, 142.0), (216.0, 141.0), (216.0, 140.0), (215.0, 139.0), (216.0, 138.0), (216.0, 137.0), (217.0, 136.0), (218.0, 135.0), (219.0, 134.0), (220.0, 133.0), (220.0, 132.0), (220.0, 131.0), (221.0, 130.0), (222.0, 129.0), (223.0, 128.0), (224.0, 127.0), (223.0, 126.0), (224.0, 125.0), (225.0, 124.0), (226.0, 123.0), (225.0, 122.0), (225.0, 121.0), (225.0, 120.0), (225.0, 119.0), (226.0, 118.0), (226.0, 117.0), (227.0, 116.0), (228.0, 115.0), (229.0, 114.0), (230.0, 113.0), (231.0, 112.0), (232.0, 111.0), (233.0, 110.0), (234.0, 109.0), (235.0, 108.0), (236.0, 107.0), (237.0, 106.0), (238.0, 105.0), (239.0, 104.0), (240.0, 103.0), (241.0, 102.0), (242.0, 101.0), (243.0, 100.0), (244.0, 99.0), (245.0, 98.0), (246.0, 97.0), (247.0, 96.0), (248.0, 95.0), (249.0, 94.0), (250.0, 93.0), (251.0, 92.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0), (252.0, 91.0)]
t2 = [(247, 170), (241, 102), (242, 101), (243, 100), (244, 99), (245, 98), (246, 97), (247, 96), (248, 95), (249, 94), (250, 93), (251, 92), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91), (252, 91)]

grid = get_grid("assets/Berlin_1_256.map")
# Initialize pygame
pygame.init()

# Constants
CELL_SIZE = 5
GRID_COLOR = (200, 200, 200)
OBSTACLE_COLOR = (0, 0, 0)
PATH_COLOR = (255, 0, 0)
OG_PATH_COLOR = (255, 255, 0)
START_COLOR = (0, 255, 0)
END_COLOR = (0, 0, 255)

# Screen dimensions
screen_width = grid.shape[1] * CELL_SIZE
screen_height = grid.shape[0] * CELL_SIZE

# Create screen
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Grid and Path Visualization")

def _is_straight_line_clear(grid, start, end):
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
        if x1 < 0 or x1 >= grid.shape[1] or y1 < 0 or y1 >= grid.shape[0] or not grid[y1, x1]:
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


def straight_path(grid, path, unsafe_zones=[]):
    """
    Strighten the path without running into obstacles. Essentially, reduce the number of jagged edges by
    checking if a straight line between two points is clear of obstacles.

    A.k.a "String pulling"
    """
    new_path = [(path[0][0], path[0][1], 0)]
    i = 0
    while i < len(path) - 1:
        j = i + 1

        while j < len(path) - 1 and _is_straight_line_clear(grid, path[i], path[j]) and path[j] != path[j-1] and j not in unsafe_zones:
            j += 1

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



# Function to draw the grid
def draw_grid():
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            color = OBSTACLE_COLOR if grid[y, x] == 1 else GRID_COLOR
            pygame.draw.rect(screen, color, rect)

# Function to draw the path
def draw_path(path, color):
    for i in range(1, len(path)):
        start = (path[i - 1][0] * CELL_SIZE + CELL_SIZE // 2, path[i - 1][1] * CELL_SIZE + CELL_SIZE // 2)
        end = (path[i][0] * CELL_SIZE + CELL_SIZE // 2, path[i][1] * CELL_SIZE + CELL_SIZE // 2)
        pygame.draw.line(screen, color, start, end, 2)

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((255, 255, 255))  # Clear screen with white background
    draw_grid()
    draw_path(t1, PATH_COLOR)
    t1 = [(int(x), int(y)) for x, y in t1]
    new_path = straight_path(grid, t1)
    print(len(t1))
    print(len(new_path))
    draw_path(new_path, OG_PATH_COLOR)
    
    pygame.draw.circle(screen, START_COLOR, (t2[0][0] * CELL_SIZE + CELL_SIZE // 2, t2[0][1] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 2)
    pygame.draw.circle(screen, END_COLOR, (t2[-1][0] * CELL_SIZE + CELL_SIZE // 2, t2[-1][1] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 2)
    pygame.display.flip()

pygame.quit()
