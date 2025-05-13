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

class GridMapWithRotation(GraphOn2DPlane):
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
        self.centers = {}
        self.rotations = {}
        rotations = [0, 90, 180, 270]  # Possible rotations in degrees

        for i in range(num_rows):
            for j in range(num_cols):
                center_x = start[0] + j * cell_size
                center_y = start[1] + i * cell_size

                for rotation in rotations:
                    node = Node(create_centered_box(center_x, center_y, cell_size, cell_size), rotation)
                    nodes.append(node)
                    rot_ind = rotations.index(rotation)
                    node_index = (i * num_cols + j) * len(rotations) + rot_ind
                    edges[node_index] = []
                    if rot_ind > 0:
                        edges[node_index].append(node_index - 1)
                    else:
                        edges[node_index].append((i * num_cols + j) * len(rotations) + len(rotations) - 1)
                    
                    if rot_ind < len(rotations) - 1:
                        edges[node_index].append(node_index + 1)
                    else:
                        edges[node_index].append((i * num_cols + j) * len(rotations))
                    
                    self.centers[node_index] = (center_x, center_y)
                    self.rotations[node_index] = rotation
    
                    # Add edges to neighboring cells with the same rotation
                    if i > 0:
                        neighbor_index = ((i - 1) * num_cols + j) * len(rotations) + rot_ind
                        edges[node_index].append(neighbor_index)
                    if i < num_rows - 1:
                        neighbor_index = ((i + 1) * num_cols + j) * len(rotations) + rot_ind
                        edges[node_index].append(neighbor_index)
                    if j > 0:
                        neighbor_index = (i * num_cols + j - 1) * len(rotations) + rot_ind
                        edges[node_index].append(neighbor_index)
                    if j < num_cols - 1:
                        neighbor_index = (i * num_cols + j + 1) * len(rotations) + rot_ind
                        edges[node_index].append(neighbor_index)

        super().__init__(nodes, edges)

    def get_rotation(self, node_id: int) -> int:
        """
        Returns the rotation of the node with the given ID.

        Args:
            node_id (int): The ID of the node.

        Returns:
            int: The rotation of the node in degrees.
        """
        if 0 <= node_id < len(self.nodes):
            return self.rotations[node_id]
        else:
            raise ValueError("Node ID out of bounds")

    def get_node_center(self, node_id: int) -> tuple[float, float]:
        """
        Returns the center coordinates of the node with the given ID.

        Args:
            node_id (int): The ID of the node.

        Returns:
            tuple[float, float]: The (x, y) coordinates of the node's center.
        """
        if 0 <= node_id < len(self.nodes):
            return self.centers[node_id]
        else:
            raise ValueError("Node ID out of bounds")
    
    def get_node_id(self, row: int, col: int, rotation: int) -> int:
        """
        Returns the node ID for the given (row, col) coordinate in the grid.

        Args:
            row (int): The row index.
            col (int): The column index.
            rotation (int): The rotation index.

        Returns:
            int: The node ID corresponding to the (row, col) coordinate.
        """
        if 0 <= row < self.num_rows and 0 <= col < self.num_cols and 0 <= rotation < len(self.rotations):
            return (row * self.num_cols + col) * len(self.rotations) + rotation
        else:
            raise ValueError("Row, column, or rotation index out of bounds")

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

        screen.fill((0, 0, 0))  # Clear the screen with a black background

        for index, graph in enumerate(graphs):
            graph.visualize(screen, colors[index])

        results = result[i % len(result)]
        for agent_id, agent_loc in enumerate(results):
            graph_id, node_id = agent_loc
            draw_circle_goal(screen, colors[graph_id], graphs[graph_id].get_node_center(goals[agent_id]), sizes[graph_id])
        for agent_id, agent_loc in enumerate(results):
            graph_id, node_id = agent_loc
            pygame.draw.circle(screen, colors[graph_id], graphs[graph_id].get_node_center(node_id), sizes[graph_id])

        pygame.time.delay(500)  # Shorter delay for interpolation
        pygame.display.flip()
        i += 1
        if i >= len(result):
            break

    pygame.quit()

trajectory = [(0,4,0), (1,7)]
trajectory2 = [(0,4,1), (1,3)]
trajectory3 = [(0,2,1), (1,4)]

graph1 = GridMap(50, 8, 8, (50, 50))
graph2 = GridMap(25, 16, 5, (252, 52))

collision_check = CollisionChecker([graph1,  graph2])
starts, ends = get_pibt_goals(collision_check, (trajectory[0], trajectory2[0], trajectory3[0]), (trajectory[-1], trajectory2[-1], trajectory3[-1]))
#starts, ends = get_pibt_goals(collision_check, (trajectory[0], trajectory2[0]), (trajectory[-1], trajectory2[-1]))

pibt_solver = PIBTFromMultiGraph(collision_check, starts, ends)
result = pibt_solver.run()
blue = (135, 206, 250)  # Light blue
red = (255, 0, 0)
#pygame.init()

# Set up the window
#width, height = 500, 500
#screen = pygame.display.set_mode((width, height))
#pygame.display.set_caption("Heterogenous PiBT visualization")
visualize_solution([graph1, graph2], (trajectory[0], trajectory2[0], trajectory3[0]), (trajectory[-1], trajectory2[-1], trajectory3[-1]), result)

"""
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
    # Interpolate between frames for smoother motion
    if i > 0:
        prev_node_id, prev_node_id2 = result[(i - 1) % len(result)]
        prev_node_id = prev_node_id[1]
        prev_node_id2 = prev_node_id2[1]

        prev_pos1 = graph1.get_node_center(prev_node_id)
        prev_pos2 = graph2.get_node_center(prev_node_id2)
        curr_pos1 = graph1.get_node_center(node_id)
        curr_pos2 = graph2.get_node_center(node_id2)

        for t in range(1, 11):  # Interpolation steps
            interp_pos1 = (
                prev_pos1[0] + (curr_pos1[0] - prev_pos1[0]) * t / 10,
                prev_pos1[1] + (curr_pos1[1] - prev_pos1[1]) * t / 10,
            )
            interp_pos2 = (
                prev_pos2[0] + (curr_pos2[0] - prev_pos2[0]) * t / 10,
                prev_pos2[1] + (curr_pos2[1] - prev_pos2[1]) * t / 10,
            )

            screen.fill((0, 0, 0))  # Clear the screen with a black background
            graph1.visualize(screen, blue)
            graph2.visualize(screen, red)

            # Draw goals
            draw_circle_goal(screen, blue, graph1.get_node_center(goal1), 10)
            draw_circle_goal(screen, red, graph2.get_node_center(goal2), 8)

            # Draw interpolated positions
            pygame.draw.circle(screen, blue, interp_pos1, 10)
            pygame.draw.circle(screen, red, interp_pos2, 8)

            pygame.display.flip()
            pygame.time.delay(50)  # Shorter delay for interpolation
    
    pygame.display.flip()
    #pygame.time.delay(500)
    i += 1
    if i >= len(result):
        i = 0

pygame.quit()
"""