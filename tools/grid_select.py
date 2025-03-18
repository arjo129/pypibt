import pygame
import numpy as np
from pypibt import get_grid
import sys

def create_bool_grid_surface(bool_grid, cell_size=1, true_color=(0, 255, 0), false_color=(255, 0, 0)):
    """Creates a Pygame surface from a numpy boolean grid."""
    rows, cols = bool_grid.shape
    surface = pygame.Surface((cols * cell_size, rows * cell_size))
    for r in range(rows):
        for c in range(cols):
            color = true_color if bool_grid[r, c] else false_color
            pygame.draw.rect(surface, color, (c * cell_size, r * cell_size, cell_size, cell_size))
            pygame.draw.rect(surface, (0,0,0), (c * cell_size, r * cell_size, cell_size, cell_size), 1) #add grid lines
    return surface

def main():
    pygame.init()

    # Example boolean grid (replace with your loaded numpy array)
    bool_grid = get_grid(sys.argv[1])

    cell_size = 5 # Adjust cell size as needed
    grid_surface = create_bool_grid_surface(bool_grid, cell_size)

    screen = pygame.display.set_mode(grid_surface.get_size())
    pygame.display.set_caption("Boolean Grid Clicker")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left mouse button
                    mouse_pos = pygame.mouse.get_pos()
                    col = mouse_pos[0] // cell_size
                    row = mouse_pos[1] // cell_size

                    if 0 <= row < bool_grid.shape[0] and 0 <= col < bool_grid.shape[1]:
                        print(f"Clicked cell: ({row}, {col}) - Value: {bool_grid[row, col]}")
                        # You can add further actions here, like highlighting the clicked cell.    

        screen.blit(grid_surface, (0, 0))
        pygame.display.flip()

    pygame.quit()   

if __name__ == "__main__":
    main()