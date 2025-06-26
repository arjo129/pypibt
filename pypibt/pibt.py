import numpy as np

from .dist_table import DistTable
from .mapf_utils import Config, Configs, Coord, Grid, get_neighbors, is_valid_coord
import heapq
import shapely
import tqdm
import collections

def is_diagonal_move(from_coord: Coord, to_coord: Coord) -> bool:
    dx = abs(from_coord[0] - to_coord[0])
    dy = abs(from_coord[1] - to_coord[1])
    return dx == 1 and dy == 1

def get_to_check_vicinity(from_coord: Coord, to_coord: Coord) -> list[Coord]:
    """
    Diagonal moves require that we check the vicinity of the destination
    """
    x_min = min(from_coord[0], to_coord[0])
    x_max = max(from_coord[0], to_coord[0])
    y_min = min(from_coord[1], to_coord[1])
    y_max = max(from_coord[1], to_coord[1])
    for y in range(y_min, y_max + 1):
        for x in range(x_min, x_max + 1):
            if (x, y) != from_coord and (x, y) != to_coord:
                yield (x, y)


class PIBT:
    def __init__(self, grid: Grid, starts: Config, goals: Config, seed: int = 0):
        self.grid = grid
        self.starts = starts
        self.goals = goals
        self.N = len(self.starts)

        # distance table
        self.dist_tables = [DistTable(grid, goal) for goal in goals]

        # cache
        self.NIL = self.N  # meaning \bot
        self.NIL_COORD: Coord = self.grid.shape  # meaning \bot
        self.occupied_now = np.full(grid.shape, self.NIL, dtype=int)
        self.occupied_nxt = np.full(grid.shape, self.NIL, dtype=int)

        # used for tie-breaking
        self.rng = np.random.default_rng(seed)

    def check_diagonal_swap_safety(self, from_coord_i: Coord, to_coord_i: Coord, i: int) -> bool:
        # Check if the swap between two agents is diagonal before running this

        for coord in get_to_check_vicinity(from_coord_i, to_coord_i):
            # Check if the next location is occupied by another agent
            if coord != from_coord_i and coord != to_coord_i:
                if is_valid_coord(self.grid, coord) and self.occupied_nxt[coord] != self.NIL:
                    return False
        return True

    def funcPIBT(self, Q_from: Config, Q_to: Config, i: int) -> bool:
        # true -> valid, false -> invalid

        # get candidate next vertices
        C = [Q_from[i]] + get_neighbors(self.grid, Q_from[i])
        self.rng.shuffle(C)  # tie-breaking, randomize
        C = sorted(C, key=lambda u: self.dist_tables[i].get(u))
        # vertex assignment
        for v in C:
            # avoid vertex collision
            if self.occupied_nxt[v] != self.NIL:
                continue

            j = self.occupied_now[v]

            # avoid edge collision
            if j != self.NIL and Q_to[j] == Q_from[i]:
                continue

            # Other type of diagonal swap conflicts also need to be avoided
            if is_diagonal_move(Q_from[i], v) and not self.check_diagonal_swap_safety(Q_from[i], v, i):
                continue

            # reserve next location
            Q_to[i] = v
            self.occupied_nxt[v] = i

            # priority inheritance (j != i due to the second condition)
            if (
                j != self.NIL
                and (Q_to[j] == self.NIL_COORD)
                and (not self.funcPIBT(Q_from, Q_to, j))
            ):
                continue

            return True

        # failed to secure node
        Q_to[i] = Q_from[i]
        self.occupied_nxt[Q_from[i]] = i
        return False

    def step(self, Q_from: Config, priorities: list[float]) -> Config:
        # setup
        N = len(Q_from)
        Q_to: Config = []
        for i, v in enumerate(Q_from):
            Q_to.append(self.NIL_COORD)
            self.occupied_now[v] = i

        # perform PIBT
        A = sorted(list(range(N)), key=lambda i: priorities[i], reverse=True)
        for i in A:
            if Q_to[i] == self.NIL_COORD:
                self.funcPIBT(Q_from, Q_to, i)

        # cleanup
        for q_from, q_to in zip(Q_from, Q_to):
            self.occupied_now[q_from] = self.NIL
            self.occupied_nxt[q_to] = self.NIL

        return Q_to

    def run(self, max_timestep: int = 1000) -> Configs:
        # define priorities
        priorities: list[float] = []
        for i in range(self.N):
            priorities.append(self.dist_tables[i].get(self.starts[i]) / self.grid.size)

        # main loop, generate sequence of configurations
        configs = [self.starts]
        while len(configs) <= max_timestep:
            # obtain new configuration
            Q = self.step(configs[-1], priorities)
            configs.append(Q)
            
            # update priorities & goal check
            flg_fin = True
            for i in range(self.N):
                if Q[i] != self.goals[i]:
                    flg_fin = False
                    priorities[i] += 1
                else:
                    priorities[i] -= np.floor(priorities[i])
            if flg_fin:
                break  # goal

        return configs


HetConfig = tuple[int, int]

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
        for index_a in tqdm.tqdm(range(len(self.nodes)), desc="Computing paths"):
            for index_b in range(len(self.nodes)):
                # Ensure we only compute each pair once (start_node_index, end_node_index)
                # and leverage symmetry.
                if index_a == index_b:
                    self.dist_cache[(index_a, index_b)] = 0
                elif (index_a, index_b) not in self.dist_cache:
                    # Use BFS for unweighted shortest paths
                    distance = self._bfs(index_a, index_b)
                    self.dist_cache[(index_a, index_b)] = distance
                    self.dist_cache[(index_b, index_a)] = distance # Cache symmetry
                    if distance != float('inf'): # Don't consider infinity for max_num
                        self.max_num = max(self.max_num, distance)
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


class CollisionChecker:
    def __init__(self, graphs: list[GraphOn2DPlane]):
        self.graphs = graphs

        # The correspondance cache keeps track of the correspondance between nodes of different graphs
        # The key is an  (int, int) tuple representing the (graph_id, node_id) of the node in the first graph
        # The value is a list of (int, int) tuples representing the (graph_id, node_id) of the corresponding nodes in the other graphs
        self.correspondence_cache = dict()
        # Precompute correspondance by iterating on all nodes of all graphs
        # This can be accelerated by collision checking structures but for
        # now do it naively
        self.precompute_correspondence()

    def get_initial_priorities(self, starts, goals):
        
        #assert len(starts) == len(self.graphs)
        #assert len(goals) == len(self.graphs)
        
        max_dist = 0
        for g in self.graphs:
            max_dist = max(max_dist, g.get_max_num())

        priorities = []
        for start_id in range(len(starts)):
            graph_id, start_node = starts[start_id]
            _, end_node = goals[start_id]
            assert _ == graph_id
            priorities.append(self.graphs[graph_id].dijkstra(start_node, end_node) / max_dist)
        return priorities
        
    def precompute_correspondence(self):
        for graph_id, graph in enumerate(self.graphs):
            for node_index, node in enumerate(graph.nodes):
                correspondence_list = [(graph_id, node_index)]
                for other_graph_id, other_graph in enumerate(self.graphs):
                    if graph_id == other_graph_id:
                        continue
                    for other_node_index, other_node in enumerate(other_graph.nodes):
                        if node.is_in_collision(other_node):
                            correspondence_list.append((other_graph_id, other_node_index))
                self.correspondence_cache[(graph_id, node_index)] = correspondence_list

    def get_other_blocked_nodes(self, graph_id: int, node_id: int) -> list[tuple[int, int]]:
        return self.correspondence_cache[(graph_id, node_id)]
    
    def get_neighbours(self, graph_id: int, node_id: int) -> list[tuple[int, int]]:
        return [(graph_id, g) for g in self.graphs[graph_id].get_neighbors(node_id)]
    
    def get_distance(self, graph_id: int, node_id: int, other_node_id: int) -> float:
        return self.graphs[graph_id].dijkstra(node_id, other_node_id)
    
    def get_null(self):
        return len(self.graphs), max([len(graph.nodes) for graph in self.graphs])
    
class ReservationSystem:
    def __init__(self, collision_checker: CollisionChecker, nil: int):
        self.current_state = dict()
        self.next_state = dict()
        self.collision_checker = collision_checker
        self.nil = nil

        for graph_id, graph in enumerate(collision_checker.graphs):
            for node_index, node in enumerate(graph.nodes):
                self.current_state[(graph_id, node_index)] = nil
                self.next_state[(graph_id, node_index)] = nil

    def clear(self):
        for graph_id, graph in enumerate(self.collision_checker.graphs):
            for node_index, node in enumerate(graph.nodes):
                self.current_state[(graph_id, node_index)] = self.nil
                self.next_state[(graph_id, node_index)] = self.nil
    
    def mark_next_state(self, graph_id: int, node_id: int, agent_id: int):
        assert isinstance(graph_id, int)
        assert isinstance(node_id, int)
        assert isinstance(agent_id, int)
        self.next_state[(graph_id, node_id)] = agent_id

    def unmark_next_state(self, graph_id: int, node_id: int, agent_id: int):
        assert isinstance(graph_id, int)
        assert isinstance(node_id, int)
        assert isinstance(agent_id, int)
        self.next_state[(graph_id, node_id)] = self.nil

    def mark_current_state(self, graph_id: int, node_id: int, agent_id: int):
        assert isinstance(graph_id, int)
        assert isinstance(node_id, int)
        assert isinstance(agent_id, int)
        self.current_state[(graph_id, node_id)] = agent_id

    def check_if_safe_to_proceed(self, graph_id: int, node_id: int, from_graph_id: int, from_node_id: int, next_config: list[HetConfig]) -> bool:
        to_check = self.collision_checker.get_other_blocked_nodes(graph_id, node_id)
        from_check = self.collision_checker.get_other_blocked_nodes(from_graph_id, from_node_id)
        for node in to_check:
            #simple vertex check
            if self.next_state[node] != self.nil:
                return False
        
        # Check if the next state of the current node is in the to_check list
        for node in to_check:
            # Check if there is an agent on any destination cell in the current state
            agent_id = self.current_state[node]
            # Check that agent does not swap with you
            if agent_id != self.nil and next_config[agent_id] in from_check:
                return False

        return True
    
    def get_currently_blocking_agents(self, graph_id: int, node_id: int) -> list[int]:
        to_check = self.collision_checker.get_other_blocked_nodes(graph_id, node_id)
        agents = []
        for node in to_check:
            #simple
            if self.current_state[node] != self.nil:
                agents.append(self.current_state[node])
        return agents
            


# For heterogenous graphs for different agens
class PIBTFromMultiGraph:
    def __init__(self, collision_checker: CollisionChecker, starts: list[HetConfig], goals: list[HetConfig], seed: int = 0):
        self.collision_checker = collision_checker
        self.starts = starts
        self.goals = goals
        self.N = len(self.starts)

        # cache
        self.NIL = self.N  # meaning \bot
        self.NIL_COORD: HetConfig = self.collision_checker.get_null()
        self.reservation_system = ReservationSystem(collision_checker, self.NIL)

        # used for tie-breaking
        self.rng = np.random.default_rng(seed)

    def funcPIBT(self, Q_from: HetConfig, Q_to: HetConfig, i: int) -> bool:
        # true -> valid, false -> invalid

        # get candidate next vertices
        C = [Q_from[i]] + self.collision_checker.get_neighbours(*Q_from[i])
        self.rng.shuffle(C)  # tie-breaking, randomize
        C = sorted(C, key=lambda u: self.collision_checker.get_distance(u[0], u[1], self.goals[i][1]))

        # vertex assignment
        for v in C:
            # avoid vertex collision
            if not self.reservation_system.check_if_safe_to_proceed(v[0], v[1], Q_from[i][0], Q_from[i][1], Q_to):
                continue

            blocking_agents = self.reservation_system.get_currently_blocking_agents(v[0], v[1])

            # reserve next location
            Q_to[i] = v
            self.reservation_system.mark_next_state(v[0], v[1], i)

            # priority inheritance (j != i due to the second condition)
            found_solution = True
            agents_affected = set()
            for agent in blocking_agents:
                if Q_to[agent] != self.NIL_COORD:
                    continue
                if self.funcPIBT(Q_from, Q_to, agent):
                    found_solution &= True
                    agents_affected.add(agent)
                else:
                    found_solution &= False
                    break
            if not found_solution:
                for agent in agents_affected:
                    self.reservation_system.unmark_next_state(*Q_to[agent], agent)
                    Q_to[agent] = self.NIL_COORD
                continue
            return True

        # failed to secure node
        Q_to[i] = Q_from[i]
        self.reservation_system.mark_next_state(Q_from[i][0], Q_from[i][1], i)
        return False

    def step(self, Q_from: list[HetConfig], priorities: list[float]) -> HetConfig:
        # setup
        N = len(Q_from)
        Q_to: Config = []
        for i, v in enumerate(Q_from):
            (graph_id, node_id) = v
            Q_to.append(self.NIL_COORD)
            self.reservation_system.mark_current_state(graph_id, node_id, i)

        # perform PIBT
        A = sorted(list(range(N)), key=lambda i: priorities[i], reverse=True)
        for i in A:
            if Q_to[i] == self.NIL_COORD:
                self.funcPIBT(Q_from, Q_to, i)

        # cleanup
        for q_from, q_to in zip(Q_from, Q_to):
            self.reservation_system.mark_current_state(q_from[0], q_from[1], self.NIL)
            self.reservation_system.mark_next_state(q_to[0], q_to[1], self.NIL)
        return Q_to

    def run(self, max_timestep: int = 1000) -> list[HetConfig]:
        # define priorities
        priorities = self.collision_checker.get_initial_priorities(self.starts, self.goals)

        # main loop, generate sequence of configurations
        configs = [self.starts]
        while len(configs) <= max_timestep:
            # obtain new configuration
            Q = self.step(configs[-1], priorities)
            configs.append(Q)

            # update priorities & goal check
            flg_fin = True
            for i in range(self.N):
                if Q[i] != self.goals[i]:
                    flg_fin = False
                    priorities[i] += 1
                else:
                    priorities[i] -= np.floor(priorities[i])
            if flg_fin:
                break  # goal

        return configs