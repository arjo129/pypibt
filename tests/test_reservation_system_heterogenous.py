from pypibt.pibt import ReservationSystemHeterogenous, CollisionChecker
from pypibt.graph_types.scaled_2d_grid import GridMap
from shapely.geometry import Polygon
import numpy as np

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

def test_space_time_bfs_with_swap():
    # Create a simple graph
    graph1 = GridMap(1.0, 3, 1, (0.5, 0.5))

    # Create a collision checker
    collision_checker = CollisionChecker([graph1])

    # Create a reservation system
    reservation_system = ReservationSystemHeterogenous(2, collision_checker)

    # Agent 1 starts at (0, 2) and moves to (0, 0)
    agent1_path = [(0, 2), (0, 1), (0, 0)]
    agent1_start_time = 0
    reservation_system.register_path(1, agent1_path, agent1_start_time)

    # Agent 0 starts at (0, 0) at time 0
    agent0_start_node = (0, 0)
    agent0_start_time = 0
    reservation_system.state[0, agent0_start_time, 0] = agent0_start_node[0]
    reservation_system.state[0, agent0_start_time, 1] = agent0_start_node[1]

    # Call space_time_bfs for agent 0
    reachable_destinations = reservation_system.space_time_bfs(0, agent0_start_time, 4)

    #  There is no way for agent 0 to progress as it would have to phaze through agent 1
    assert (0,2) not in reachable_destinations

def test_space_time_bfs_with_rollback():
    # Create a simple graph
    graph1 = GridMap(1.0, 3, 3, (0.5, 0.5))

    # Create a collision checker
    collision_checker = CollisionChecker([graph1])

    # Create a reservation system
    reservation_system = ReservationSystemHeterogenous(2, collision_checker)

    # Set agent 0's start
    agent0_start_node = (0, 0)
    agent0_start_time = 0
    reservation_system.state[0, agent0_start_time, 0] = agent0_start_node[0]
    reservation_system.state[0, agent0_start_time, 1] = agent0_start_node[1]

    # Call space_time_bfs for agent 0
    initial_reachable_destinations = reservation_system.space_time_bfs(0, agent0_start_time, 4)

    # Consider a new path for agent 1 that blocks some destinations
    agent1_path = [(0, 1), (0, 2), (0, 5)]
    agent1_start_time = 1
    reservation_system.consider_new_path(1, agent1_path, agent1_start_time)

    # Call space_time_bfs for agent 0 again
    reachable_destinations_with_considered_path = reservation_system.space_time_bfs(0, agent0_start_time, 4)

    # Assert that the considered path blocks some destinations
    assert len(reachable_destinations_with_considered_path) < len(initial_reachable_destinations)

    # Roll back the considered path
    reservation_system.roll_back_paths_from(1)

    # Call space_time_bfs for agent 0 one more time
    reachable_destinations_after_rollback = reservation_system.space_time_bfs(0, agent0_start_time, 4)

    # Assert that the reachable destinations are the same as the initial ones
    assert len(reachable_destinations_after_rollback) == len(initial_reachable_destinations)
    assert sorted(list(reachable_destinations_after_rollback)) == sorted(list(initial_reachable_destinations))

def test_commit_path():
    # Create a simple graph
    graph1 = GridMap(1.0, 3, 3, (0.5, 0.5))

    # Create a collision checker
    collision_checker = CollisionChecker([graph1])

    # Create a reservation system
    reservation_system = ReservationSystemHeterogenous(2, collision_checker)

    # Consider a new path for agent 1
    agent1_path = [(0, 1), (0, 2), (0, 5)]
    agent1_start_time = 1
    reservation_system.consider_new_path(1, agent1_path, agent1_start_time)

    # Commit the path
    reservation_system.commit_paths()

    # Assert that the correct nodes are blocked at the correct times
    for i, (graph_id, node_id) in enumerate(agent1_path):
        time = agent1_start_time + i
        # Check that the node is blocked
        assert (graph_id, node_id) in reservation_system.blocked_nodes[time]
        # Check that it is blocked by the correct agent
        assert reservation_system.blocked_nodes[time][(graph_id, node_id)] == 1
        # Check that the state is updated
        assert reservation_system.state[1, time, 0] == graph_id
        assert reservation_system.state[1, time, 1] == node_id

    # Assert that the "in consideration" data structures are empty
    assert len(reservation_system.paths_in_consideration) == 0
    assert len(reservation_system.consideration_order) == 0
    assert len(reservation_system.considering_agents) == 0
    assert len(reservation_system.considered_blocked_nodes_by_agent) == 0
    for t in range(len(reservation_system.considering_to_block)):
        assert len(reservation_system.considering_to_block[t]) == 0