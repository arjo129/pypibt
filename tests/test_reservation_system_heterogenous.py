from pypibt.pibt import ReservationSystemHeterogenous, CollisionChecker
from pypibt.graph_types.scaled_2d_grid import GridMap
from shapely.geometry import Polygon
import numpy as np

def create_centered_box(center_x: float, center_y: float, width: float, height: float) -> Polygon:
    """
    Creates a shapely Polygon representing a box centered at (center_x, center_y).
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

def test_space_time_bfs():
    # Create a simple graph
    graph1 = GridMap(1.0, 3, 3, (0.5, 0.5))

    # Create a collision checker
    collision_checker = CollisionChecker([graph1])

    # Create a reservation system
    reservation_system = ReservationSystemHeterogenous(2, collision_checker)

    # Register a path for agent 1
    agent1_path = [(0, 1), (0, 2), (0, 5)]
    agent1_start_time = 1
    reservation_system.register_path(1, agent1_path, agent1_start_time)

    # Assert that the correct nodes are blocked at the correct times
    for i, (graph_id, node_id) in enumerate(agent1_path):
        time = agent1_start_time + i
        # Check that the node is blocked
        assert (graph_id, node_id) in reservation_system.blocked_nodes[time]
        # Check that it is blocked by the correct agent
        assert reservation_system.blocked_nodes[time][(graph_id, node_id)] == 1

    # Set agent 0's start
    agent0_start_node = (0, 0)
    agent0_start_time = 0
    reservation_system.state[0, agent0_start_time, 0] = agent0_start_node[0]
    reservation_system.state[0, agent0_start_time, 1] = agent0_start_node[1]

    # Call space_time_bfs for agent 0
    reachable_destinations = reservation_system.space_time_bfs(0, agent0_start_time, 4)

    # Assert that the reachable destinations do not collide with agent 1's path
    for pos, travel_time in reachable_destinations:
        absolute_time = agent0_start_time + travel_time
        # Check if the position is blocked at the given time
        assert pos not in reservation_system.blocked_nodes[absolute_time]
