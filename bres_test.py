import pygame
import numpy as np

def thick_bresenham_line_stop_on_obstacle(p1, p2, grid, thickness=1):
    """Draws a thick Bresenham line and stops when an obstacle is encountered."""
    x1, y1 = p1
    x2, y2 = p2

    main_line = np.linspace((x1, y1), (x2, y2), num=max(abs(x2 - x1), abs(y2 - y1)) + 1).astype(int)

    points_to_draw = []  # Store points to draw before stopping
    for x, y in main_line:
        for dx in range(-thickness, thickness + 1):
            for dy in range(-thickness, thickness + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]:
                    if not grid[nx, ny]:  # Obstacle detected
                        return points_to_draw #stop drawing
                    else:
                        points_to_draw.append((nx,ny))
    return points_to_draw

def visualize_bresenham(grid, p1, p2, thickness):
    """Visualizes the thick Bresenham line and obstacles in Pygame."""
    pygame.init()

    # Determine window size based on grid dimensions
    width = grid.shape[1] * 10
    height = grid.shape[0] * 10
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Thick Bresenham Line")

    # Colors
    white = (255, 255, 255)
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)

    # Draw grid and obstacles
    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):
            if not grid[x, y]:  # Obstacle
                pygame.draw.rect(screen, black, (y * 10, x * 10, 10, 10))
            else:
                pygame.draw.rect(screen, white, (y * 10, x * 10, 10, 10), 1)

    # Draw start and end points
    pygame.draw.circle(screen, green, (p1[1] * 10 + 5, p1[0] * 10 + 5), 5)
    pygame.draw.circle(screen, blue, (p2[1] * 10 + 5, p2[0] * 10 + 5), 5)

    # Get the points to draw, stopping at the obstacle
    points = thick_bresenham_line_stop_on_obstacle(p1, p2, grid, thickness)
    for x, y in points:
        pygame.draw.rect(screen, red, (y * 10, x * 10, 10, 10))

    pygame.display.flip()


    pygame.display.flip()

    # Event loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

# Example usage:
grid_size = (50, 50)
grid = np.full(grid_size, True)

# Create some obstacles
for i in range(10, 40):
    grid[i, 25] = False

p1 = (5, 5)
p2 = (45, 45)
thickness = 3

visualize_bresenham(grid, p1, p2, thickness)