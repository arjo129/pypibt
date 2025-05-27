from pypibt import Node, GraphOn2DPlane
from pypibt.mapf_utils import Grid
from shapely.geometry import Polygon

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
        self.centers = {}

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
                self.centers[node_index] =(center_x, center_y)

        super().__init__(nodes, edges)

    def get_swept_collision(self, from_node, to_node):
        pass

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

class StaticObstacle:
    def __init__(self, cell_size: int, occupancy_map: Grid):
        self.cell_size = cell_size
        self.occupancy_map = occupancy_map

    def get_occupancy_at(self, x,y):
        ind_x, ind_y = int(x//self.cell_size), int(y//self.cell_size)
        return self.occupancy_map[ind_x, ind_y]

    def visuallize(self, screen, color):
        if 'pygame' not in globals():
            import pygame
        pygame.draw.rect(screen, color, )

    def is_safe_location(self, min_x: float, min_y: float,
            max_x: float, max_y: float):
        """
        Iterates over a rectangular region in real space and prints the occupancy
        of each cell within that region.

        Args:
            min_x, min_y: The bottom-left real-space coordinates of the region.
            max_x, max_y: The top-right real-space coordinates of the region.
        """

        cell_size = self.cell_size

        # Convert real-space boundaries to grid indices
        start_ind_x = int(min_x // cell_size)
        start_ind_y = int(min_y // cell_size)

        end_ind_x = int(max_x // cell_size)
        end_ind_y = int(max_y // cell_size)

        # Ensure indices are within map bounds
        start_ind_x = max(0, start_ind_x)
        start_ind_y = max(0, start_ind_y)
        end_ind_x = min(self.occupancy_map.shape[0] - 1, end_ind_x)
        end_ind_y = min(self.occupancy_map.shape[1] - 1, end_ind_y)

        #print(f"Iterating from grid cell ({start_ind_x}, {start_ind_y}) to ({end_ind_x}, {end_ind_y})")

        # Iterate through the relevant grid cells
        for ind_x in range(start_ind_x, end_ind_x + 1):
            for ind_y in range(start_ind_y, end_ind_y + 1):
                # Calculate real-space coordinates for the center of the current cell
                # This is one way to get a representative point for the cell.
                # You could also use (ind_x * cell_size, ind_y * cell_size) for the bottom-left corner.
                real_x = ind_x * cell_size + cell_size / 2
                real_y = ind_y * cell_size + cell_size / 2

                if not self.get_occupancy_at(real_x, real_y):
                    return False
        return True


class GridMapWithStaticObstacles(GridMap):
    def __init__(self, cell_size: float, num_rows: int, num_cols: int, start: tuple[float], static_obstacles: StaticObstacle):
        self.static_obstacles = static_obstacles
        super().__init__(cell_size, num_rows=num_rows, num_cols=num_cols, start=start)

    def get_neighbors(self, node_index: int) -> list[int]:
        neighbours = super().get_neighbors(node_index)
        result = []
        for neigh in neighbours:
            x,y = self.get_node_center(node_index)
            top_left = (x - self.cell_size/2, y - self.cell_size/2)
            bottom_right = (x + self.cell_size/2, y + self.cell_size/2)
            if self.static_obstacles.is_safe_location(top_left[0], top_left[1], bottom_right[0], bottom_right[1]):
                result.append(neigh)
        return result