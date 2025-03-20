from pypibt import Node, GraphOn2DPlane, CollisionChecker, ReservationSystem
from shapely.geometry import Polygon
import pygame

def create_centered_box(center_x: float, center_y: float, width: float, height: float) -> Polygon:
    """
    Creates a shapely Polygon representing a box centered at (center_x, center_y).

    Args:
        center_x (float): The x-coordinate of the center.
        center_y (float): The y-coordinate of the center.
        width (float): The width of the box.
        height (float): The height of the box.

    Returns:
        shapely.geometry.Polygon: A Polygon object representing the box.
    """
    half_width = width / 2.0
    half_height = height / 2.0

    # Calculate the coordinates of the corners
    x1 = center_x - half_width
    y1 = center_y - half_height
    x2 = center_x + half_width
    y2 = center_y - half_height
    x3 = center_x + half_width
    y3 = center_y + half_height
    x4 = center_x - half_width
    y4 = center_y + half_height

    # Create the Polygon
    polygon = Polygon([(x1, y1), (x2, y2), (x3, y3), (x4, y4)])
    return polygon

def create_grid(cell_size: float, num_rows: int, num_cols: int, start: tuple[float]) -> GraphOn2DPlane:
    """
    Creates a grid graph with the given cell size, number of rows, and number of columns.

    Args:
        cell_size (float): The size of each cell.
        num_rows (int): The number of rows in the grid.
        num_cols (int): The number of columns in the grid.
        start (tuple[float]): The starting position of the grid.

    Returns:
        GraphOn2DPlane: A grid graph.
    """
    nodes = []
    edges = {}
    for i in range(num_rows):
        for j in range(num_cols):
            center_x = start[0] + j * cell_size
            center_y = start[1] + i * cell_size
            node = Node(create_centered_box(center_x, center_y, cell_size, cell_size))
            nodes.append(node)
            node_index = i * num_cols + j
            edges[node_index] = []
            if i > 0:
                edges[node_index].append((i - 1) * num_cols + j)
            if i < num_rows - 1:
                edges[node_index].append((i + 1) * num_cols + j)
            if j > 0:
                edges[node_index].append(i * num_cols + j - 1)
            if j < num_cols - 1:
                edges[node_index].append(i * num_cols + j + 1)
    return GraphOn2DPlane(nodes, edges)


graph1 = create_grid(50, 8, 8, (50, 50))
graph2 = create_grid(50, 10, 10, (250, 50))

blue = (135, 206, 250)  # Light blue
red = (255, 0, 0)
pygame.init()

# Set up the window
width, height = 400, 400
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Hererogenous PiBT visuallization")
graph1.visualize(screen, blue)
graph2.visualize(screen, red)
pygame.display.flip()
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()