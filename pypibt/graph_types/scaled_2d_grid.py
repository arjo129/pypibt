from pypibt.graph_types.base2d_grid import GraphOn2DPlane, Node
from pypibt.mapf_utils import Grid
from shapely.geometry import Polygon, box
import random
import numpy as np
import tqdm

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

def _clamp(val, min_v, max_v):
    return min(max(min_v, val), max_v)

def scale_up_grid(grid, i):
    """
    Scales up a 2D NumPy grid by an integer factor 'i'.

    Each element of the original grid is repeated 'i' times both
    horizontally and vertically.

    Args:
        grid (np.ndarray): The input 2D NumPy array (m x n).
        i (int): The integer scaling factor. Must be a positive integer.

    Returns:
        np.ndarray: The scaled-up 2D NumPy array ((m*i) x (n*i)).

    Raises:
        ValueError: If 'i' is not a positive integer.
    """
    if not isinstance(i, int) or i <= 0:
        raise ValueError("Scaling factor 'i' must be a positive integer.")

    # Get the dimensions of the original grid
    m, n = grid.shape

    # Create an empty grid with the new dimensions
    scaled_grid = np.zeros((m * i, n * i), dtype=grid.dtype)

    # Populate the scaled grid
    for r in range(m):
        for c in range(n):
            scaled_grid[r*i : (r+1)*i, c*i : (c+1)*i] = grid[r, c]

    return scaled_grid

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
        self.row_cols = {}

        nodes = []
      
        edges = {}
        for i in tqdm.tqdm(range(num_rows)):
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
                self.row_cols[node_index] = (i,j)

        super().__init__(nodes, edges)

    def get_cost(self, node_from, node_to):
        return super().get_cost(node_from, node_to) * self.cell_size

    def get_corners(self, node_id):
        center_x, center_y = self.get_node_center(node_id)
        top_left_x, top_left_y = center_x - self.cell_size/2, center_y - self.cell_size/2
        bottom_right_x, bottom_right_y = center_x + self.cell_size/2, center_y + self.cell_size/2
        return (top_left_x, top_left_y), (bottom_right_x, bottom_right_y)

    def get_swept_collision(self, from_node, to_node):
        pass

    def from_node_center_to_node_id(self, x:float, y: float):
        j,i = int((x - self.start[0])/self.cell_size) , int((y - self.start[1])/self.cell_size)
        return i * self.num_cols + j

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
        self.upscaled_map = scale_up_grid(occupancy_map, cell_size)


    def visualize(self, screen, color, scale=1):
        if 'pygame' not in globals():
            import pygame

        for x in range(self.occupancy_map.shape[0]):
            for y in range(self.occupancy_map.shape[1]):
                if self.occupancy_map[x, y]:
                    continue
                pygame.draw.rect(screen, color, (x * self.cell_size*scale, y * self.cell_size*scale, self.cell_size*scale, self.cell_size*scale))

    def is_safe_location(self, min_x: float, min_y: float,
            max_x: float, max_y: float):
        min_x = int(min_x)
        min_y = int(min_y)
        max_x = int(max_x)
        max_y = int(max_y)

        min_x = _clamp(min_x, 0, self.upscaled_map.shape[0])
        min_y = _clamp(min_y, 0, self.upscaled_map.shape[1])
        max_x = _clamp(max_x, 0, self.upscaled_map.shape[0])
        max_y = _clamp(max_y, 0, self.upscaled_map.shape[1])

        start_ind_x = min(min_x, max_x)
        end_ind_x = max(min_x, max_x)

        start_ind_y = min(min_y, max_y)
        end_ind_y = max(min_y, max_y)

        for ind_x in range(start_ind_x, end_ind_x):
            for ind_y in range(start_ind_y, end_ind_y):
                if not self.upscaled_map[ind_x, ind_y]:
                    return False
        return True


class GridMapWithStaticObstacles(GridMap):
    def __init__(self, cell_size: float, num_rows: int, num_cols: int, start: tuple[float], static_obstacles: StaticObstacle):
        self.static_obstacles = static_obstacles
        self.neighbor_cache = {}

        super().__init__(cell_size, num_rows=num_rows, num_cols=num_cols, start=start)

    def visualize(self, screen, color, scale= 1):
        super().visualize(screen, color, scale)
        if 'pygame' not in globals():
            import pygame

        #for node in range(len(self.nodes)):
        #    if not self.is_safe_node(node):
        #        tl, _= self.get_corners(node)
        #        pygame.draw.rect(screen, color, (tl[0] * scale, tl[1] * scale, self.cell_size *scale, self.cell_size *scale))

    def to_numpy_array(self):
        arr = np.full((self.num_rows, self.num_cols), False, dtype=bool)
        for node in range(len(self.nodes)):
            if self.is_safe_node(node):
                r,c = self.row_cols[node]
                arr[r,c] = True
        return arr

    def get_neighbors(self, node_index: int) -> list[int]:
        if node_index in self.neighbor_cache:
            return self.neighbor_cache[node_index]
        neighbours = super().get_neighbors(node_index)
        result = []
        if not self.is_safe_node(node_index):
            self.neighbor_cache[node_index] = []
            return []
        for neigh in neighbours:
            if self.is_safe_node(neigh):
                result.append(neigh)

        self.neighbor_cache[node_index] = result
        return result

    def is_safe_node(self, node_index):
        x,y = self.get_node_center(node_index)
        top_left = (x - self.cell_size/2, y - self.cell_size/2)
        bottom_right = (x + self.cell_size/2, y + self.cell_size/2)
        return self.static_obstacles.is_safe_location(top_left[0], top_left[1], bottom_right[0], bottom_right[1])
