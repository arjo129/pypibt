from pypibt import Node, GraphOn2DPlane, CollisionChecker, ReservationSystem
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



def test_graph_collision_check():
    node1 = Node(create_centered_box(0.5,0.5,1,1))
    node2 = Node(create_centered_box(1.5,1.5,1,1))

    graph1 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})

    node3 = Node(create_centered_box(1.0,1.0,1,1))

    graph2 = GraphOn2DPlane([node3], {})

    common_collision = CollisionChecker([graph1, graph2])

    assert common_collision.get_other_blocked_nodes(0,0) == [(1,0)]
    assert common_collision.get_other_blocked_nodes(0,1) == [(1,0)]
    assert common_collision.get_other_blocked_nodes(1,0) == [(0,0), (0,1)]

def test_swap_check():
    node1 = Node(create_centered_box(0.5,0.5,1,1))
    node2 = Node(create_centered_box(1.5,1.5,1,1))

    graph1 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})
    graph2 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})

    common_collision = CollisionChecker([graph1, graph2])
    reservation_system = ReservationSystem(common_collision, 5)
    reservation_system.mark_current_state(0,0,0)
    reservation_system.mark_current_state(1,1,1)
    assert reservation_system.check_if_safe_to_proceed(0,1,0,0) == False

def test_ok_check():
    node1 = Node(create_centered_box(0.5,0.5,1,1))
    node2 = Node(create_centered_box(1.5,1.5,1,1))

    graph1 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})
    graph2 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})

    common_collision = CollisionChecker([graph1, graph2])
    reservation_system = ReservationSystem(common_collision, 5)
    reservation_system.mark_current_state(0,0,0)
    assert reservation_system.check_if_safe_to_proceed(0,1,0,0)

def test_geometry_check():
    node1 = Node(create_centered_box(0.5,0.5,1,1))
    node2 = Node(create_centered_box(1.5,1.5,1,1))
    node3 = Node(create_centered_box(2.5,2.5,1,1))

    graph1 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})
    graph2 = GraphOn2DPlane([node3, node2], {0:[1], 1:[0]})

    common_collision = CollisionChecker([graph1, graph2])
    reservation_system = ReservationSystem(common_collision, 5)
    reservation_system.mark_current_state(0,0,0)
    reservation_system.mark_current_state(1,0,1)

    assert reservation_system.check_if_safe_to_proceed(0,1,0,0)

    reservation_system.mark_next_state(0,1,0)
    assert reservation_system.check_if_safe_to_proceed(1,1,1,0) == False
#test_graph_collision_check()
#test_geometry_check()