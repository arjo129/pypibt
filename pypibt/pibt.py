import numpy as np

from .dist_table import DistTable
from .mapf_utils import Config, Configs, Coord, Grid, get_neighbors
import heapq


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

class Node:
    def __init__(self, coord: tuple[float, float], size: float):
        self.coord = coord
        self.size = size

    def is_in_collision(self, other: "Node") -> bool:
        # Check if two square nodes are in collision
        dx = abs(self.coord[0] - other.coord[0])
        dy = abs(self.coord[1] - other.coord[1])
        return dx < (self.size + other.size) / 2 and dy < (self.size + other.size) / 2
"""
Representation of a nav mesh graph on a 2D plane
"""
class GraphOn2DPlane:
    def __init__(self, nodes: list[Node], neighbors: dict[int, list[int]]):
        self.nodes = nodes
        self.neighbors = neighbors
        self.dist_cache = dict()
        # Build distance cache for all pairs of nodes
        for index_a, node_a in enumerate(self.nodes):
            for index_b, node_b in enumerate(self.nodes):
                if index_a != index_b:
                    self.dist_cache[(index_a, index_b)] = self.dijkstra(index_a, index_b)

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
            for neighbor in self.neighbors[current_node]:
                if neighbor in visited:
                    continue
                new_distance = current_distance + 1  # Assuming uniform edge cost
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    heapq.heappush(pq, (new_distance, neighbor))

        # Cache the result
        self.dist_cache[(start_node_index, end_node_index)] = distances[end_node_index]
        return distances[end_node_index]

    def get_neighbors(self, node_index: int) -> list[int]:
        return self.neighbors[node_index]

class CollisionChecker:
    def __init__(self, graphs: list[GraphOn2DPlane]):
        self.graphs = graphs

        # The correspondance cache keeps track of the correspondance between nodes of different graphs
        # The key is an  (int, int) tuple representing the (graph_id, node_id) of the node in the first graph
        # The value is a list of (int, int) tuples representing the (graph_id, node_id) of the corresponding nodes in the other graphs
        self.correspondance_cache = dict()
        # Precompute correspondance by iterating on all nodes of all graphs
        # This can be accelerated by collision checking structures but for
        # now do it naively
        self.precompute_correspondance()

    def precompute_correspondance(self):
        for graph_id, graph in enumerate(self.graphs):
            for node_index, node in enumerate(graph.nodes):
                correspondance_list = []
                for other_graph_id, other_graph in enumerate(self.graphs):
                    if graph_id == other_graph_id:
                        continue
                    for other_node_index, other_node in enumerate(other_graph.nodes):
                        if node.is_in_collision(other_node):
                            correspondance_list.append((other_graph_id, other_node_index))
                self.correspondance_cache[(graph_id, node_index)] = correspondance_list

    def get_other_blocked_nodes(self, graph_id: int, node_id: int) -> list[tuple[int, int]]:
        return self.correspondance_cache[(graph_id, node_id)]
    
    def get_neighbours(self, graph_id: int, node_id: int) -> list[tuple[int, int]]:
        return self.graphs[graph_id].get_neighbors(node_id)
    
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
    
    def mark_next_state(self, graph_id: int, node_id: int, agent_id: int):
        self.next_state[(graph_id, node_id)] = agent_id

    def mark_current_state(self, graph_id: int, node_id: int, agent_id: int):
        self.current_state[(graph_id, node_id)] = agent_id

    def check_if_safe_to_proceed(self, graph_id: int, node_id: int) -> bool:
        # equivalent of vertex 
        to_check = self.collision_checker.get_other_blocked_nodes(graph_id, node_id)
        for node in to_check:
            #simple
            if self.next_state[node] != self.nil:
                return False
            


# For heterogenous graphs for different agens
class PIBTFromMultiGraph:
    def __init__(self, collision_checker: CollisionChecker, starts: list[HetConfig], goals: list[HetConfig], seed: int = 0):
        self.collision_checker = collision_checker
        self.starts = starts
        self.goals = goals
        self.N = len(self.starts)

        # cache
        self.NIL = self.N  # meaning \bot
        self.NIL_COORD: Coord = self.grid.shape  # meaning \bot
        self.occupied_now = np.full(grid.shape, self.NIL, dtype=int)
        self.occupied_nxt = np.full(grid.shape, self.NIL, dtype=int)

        # used for tie-breaking
        self.rng = np.random.default_rng(seed)

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