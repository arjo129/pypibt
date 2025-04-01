from pypibt import Node, GraphOn2DPlane, CollisionChecker, PIBTFromMultiGraph
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



class GridMap(GraphOn2DPlane):
    def __init__(self, cell_size: float, num_rows: int, num_cols: int, start: tuple[float]):
        """
        Initializes a GridMap object.

        Args:
            cell_size (float): The size of each cell.
            num_rows (int): The number of rows in the grid.
            num_cols (int): The number of columns in the grid.
            start (tuple[float]): The starting position of the grid.
        """
        self.cell_size = cell_size
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.start = start

        nodes = []
      
        edges = {}
        for i in range(num_rows):
            for j in range(num_cols):
                center_x = start[0] + j * cell_size
                center_y = start[1] + i * cell_size
                node = Node(create_centered_box(center_x, center_y, cell_size, cell_size))
                nodes.append(node)
                node_index = i * num_cols + j
                #print(node_index)
                edges[node_index] = []
                if i > 0:
                    edges[node_index].append((i - 1) * num_cols + j)
                if i < num_rows - 1:
                    edges[node_index].append((i + 1) * num_cols + j)
                if j > 0:
                    edges[node_index].append(i * num_cols + j - 1)
                if j < num_cols - 1:
                    edges[node_index].append(i * num_cols + j + 1)

        super().__init__(nodes, edges)

    def get_node_center(self, node_id: int) -> tuple[float, float]:
        """
        Returns the center coordinates of the node with the given ID.

        Args:
            node_id (int): The ID of the node.

        Returns:
            tuple[float, float]: The (x, y) coordinates of the node's center.
        """
        if 0 <= node_id < len(self.nodes):
            col = node_id % self.num_cols
            row = node_id // self.num_cols
            center_x = self.start[0] + col * self.cell_size
            center_y = self.start[1] + row * self.cell_size
            return center_x, center_y
        else:
            raise ValueError("Node ID out of bounds")

    def get_node_id(self, row: int, col: int) -> int:
        """
        Returns the node ID for the given (row, col) coordinate in the grid.

        Args:
            row (int): The row index.
            col (int): The column index.

        Returns:
            int: The node ID corresponding to the (row, col) coordinate.
        """
        if 0 <= row < self.num_rows and 0 <= col < self.num_cols:
            return row * self.num_cols + col
        else:
            raise ValueError("Row or column index out of bounds")

def get_pibt_goals(collision_checker: CollisionChecker, start_coords, end_coords):
    graphs = collision_checker.graphs
    start_configs = []
    end_configs = []
    for i in range(len(graphs)):
        g = graphs[i]
        start_node_id = g.get_node_id(*start_coords[i])
        end_node_id = g.get_node_id(*end_coords[i])
        start_configs.append((i, start_node_id))
        end_configs.append((i, end_node_id))
    return start_configs, end_configs
    
trajectory = [(0,4), (1,7)]
trajectory2 = [(0,4), (1,3)]

graph1 = GridMap(50, 8, 8, (50, 50))
graph2 = GridMap(25, 16, 5, (252, 52))

collision_check = CollisionChecker([graph1,  graph2])
starts, ends = get_pibt_goals(collision_check, (trajectory[0], trajectory2[0]), (trajectory[-1], trajectory2[-1]))
pibt_solver = PIBTFromMultiGraph(collision_check, starts, ends)
result = pibt_solver.run()
blue = (135, 206, 250)  # Light blue
red = (255, 0, 0)
pygame.init()

# Set up the window
width, height = 500, 500
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Hererogenous PiBT visuallization")



goal1 = graph1.get_node_id(*trajectory[-1])
goal2 = graph2.get_node_id(*trajectory2[-1])
pygame.display.flip()
running = True
i = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))  # Clear the screen with a black background
    graph1.visualize(screen, blue)
    graph2.visualize(screen, red)
    node_id, node_id2 = result[i % len(result)]
    node_id = node_id[1]
    node_id2 = node_id2[1]
    
    # Draw goals first
    draw_circle_goal(screen, blue, graph1.get_node_center(goal1), 10)
    draw_circle_goal(screen, red, graph2.get_node_center(goal2), 8)

    pygame.draw.circle(screen, blue, graph1.get_node_center(node_id), 10)
    pygame.draw.circle(screen, red, graph2.get_node_center(node_id2), 8)
    
    
    pygame.display.flip()
    pygame.time.delay(500)
    i += 1

pygame.quit()