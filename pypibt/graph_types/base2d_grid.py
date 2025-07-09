import shapely
import tqdm
import collections
"""
Every node in a graph has a shape property. The shape property represents the shape of the node in 2D space.
"""
class Node:
    def __init__(self, shapely_shape: shapely.geometry.Polygon):
        self.shape = shapely_shape

    def is_in_collision(self, other: "Node") -> bool:
        # Check if two square nodes are in collision
        return self.shape.intersects(other.shape)
    
    def visualize(self, screen, color):
        if 'pygame' not in globals():
            import pygame
        # Draw the outline of the polygon by iterating over its edges
        for i in range(len(self.shape.exterior.coords) - 1):
            start = self.shape.exterior.coords[i]
            end = self.shape.exterior.coords[i + 1]
            pygame.draw.line(screen, color, start, end, 1)

"""
Representation of a nav mesh graph on a 2D plane
"""
class GraphOn2DPlane:
    def __init__(self, nodes: list[Node], neighbors: dict[int, list[int]]):
        self.nodes = nodes
        self.neighbors = neighbors
        self.dist_cache = dict()
        self.max_num = 0
        # Build distance cache for all pairs of nodes
        """
        for index_a, node_a in tqdm.tqdm(enumerate(self.nodes)):
            for index_b, node_b in enumerate(self.nodes):
                if index_a != index_b:
                    self.dist_cache[(index_a, index_b)] = self.dijkstra(index_a, index_b)
                    self.max_num = max(self.max_num, self.dist_cache[(index_a, index_b)])
        """
        # Pre-compute all-pairs shortest paths using a modified BFS
        # This loop now just initiates the BFS for each starting node
        for start_node_index in tqdm.tqdm(range(len(self.nodes)), desc="Computing paths"):
            self._bfs_all_from_source(start_node_index)

        # After all BFS runs, compute max_num from the populated dist_cache
        # We need to iterate through the cache to find the maximum
        for (idx_a, idx_b), dist in self.dist_cache.items():
            if dist != float('inf'):
                self.max_num = max(self.max_num, dist)

    def get_max_num(self):
        return self.max_num

    def visualize(self, screen, color):
        for node in self.nodes:
            node.visualize(screen, color)

    def get_cost(self, node_from, node_to):
        return 1

    def dijkstra(self, start_node_index: int, end_node_index: int) -> float:
        if (start_node_index, end_node_index) in self.dist_cache:
            return self.dist_cache[(start_node_index, end_node_index)]

        if start_node_index >= len(self.nodes)  or end_node_index >= len(self.nodes):
            raise Exception(f"Invalid node name {start_node_index} -> {end_node_index}, max nodes {len(self.nodes)}")

        # Initialize distances and visited set
        distances = {index: float('inf') for index in range(len(self.nodes))}
        distances[start_node_index] = 0
        visited = set()

        # Priority queue for Dijkstra's algorithm
        pq = [(0, start_node_index)]  # (distance, node_id)

        while pq:
            current_distance, current_node = heapq.heappop(pq)

            if current_node in visited:
                continue
            visited.add(current_node)

            # Early exit if we reach the target node
            if current_node == end_node_index:
                break

            # Update distances for neighbors
            for neighbor in self.get_neighbors(current_node):
                if neighbor in visited:
                    continue
                new_distance = current_distance + self.get_cost(current_node, neighbor)  # Assuming uniform edge cost
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(pq, (new_distance, neighbor))

        # Cache the result
        self.dist_cache[(start_node_index, end_node_index)] = distances[end_node_index]

        # Symmetry to reduce duplication
        self.dist_cache[(end_node_index, start_node_index)] = distances[end_node_index]
        return distances[end_node_index]

    def _bfs_all_from_source(self, start_node_index: int):
        """
        Modified BFS to compute shortest paths from a single source to all other reachable nodes,
        and populate the dist_cache along the way.

        Args:
            start_node_index: The index of the starting node.
        """
        # Ensure the start node to itself is 0
        self.dist_cache[(start_node_index, start_node_index)] = 0

        queue = collections.deque([(start_node_index, 0)])
        # Visited set is important for BFS correctness, but we also populate cache
        # The cache itself will act as a visited check for future traversals from this source
        # For a single source BFS, `current_distance` is the shortest.

        # We'll use a local 'distances' dictionary for the current BFS run
        # This ensures we don't interfere with a partially built global cache
        # for other start nodes that might be running in a multi-threaded scenario (though not here)
        # and it also allows us to build up the distances for the current source

        # Using dist_cache directly as the visited set for this specific source BFS run
        # is a more direct approach as well.

        # Using a set to track visited nodes for the *current* BFS run
        visited_in_current_bfs = {start_node_index}

        while queue:
            current_node, current_distance = queue.popleft()

            # Populate the cache for this pair
            # This is safe because BFS explores in layers, so the first time we reach
            # a node from start_node_index, it's the shortest path.
            self.dist_cache[(start_node_index, current_node)] = current_distance * self.get_cost(0,0)
            self.dist_cache[(current_node, start_node_index)] = current_distance * self.get_cost(0,0) # Leverage symmetry

            for neighbor in self.get_neighbors(current_node):
                if neighbor not in visited_in_current_bfs:
                    visited_in_current_bfs.add(neighbor)
                    queue.append((neighbor, current_distance + 1))

    def _bfs(self, start_node_index: int, end_node_index: int) -> float:
        """
        Private helper method to compute the shortest path distance using Breadth-First Search.
        This is suitable for unweighted graphs (where all edge costs are 1).

        Args:
            start_node_index: The index of the starting node.
            end_node_index: The index of the destination node.

        Returns:
            The shortest distance between the start and end nodes.
            Returns float('inf') if no path exists.
        """
        if start_node_index == end_node_index:
            return 0

        # Queue for BFS: stores (node_id, distance)
        queue = collections.deque([(start_node_index, 0)])
        visited = {start_node_index}

        while queue:
            current_node, current_distance = queue.popleft()

            # If we've reached the target node, return the distance
            if current_node == end_node_index:
                return current_distance

            # Explore unvisited neighbors
            for neighbor in self.get_neighbors(current_node):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, current_distance + 1)) # Increment distance by 1 for uniform cost

        return float('inf') 

    def get_neighbors(self, node_index: int) -> list[int]:
        return self.neighbors[node_index]
    
    def get_swept_collision(self, from_node: int, to_node: int) -> shapely.geometry.Polygon:
        pass
