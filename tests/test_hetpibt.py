from pypibt import Node, GraphOn2DPlane, CollisionChecker


def test_node_collision_check():
    node1 = Node((0,0), 0.5)
    node2 = Node((1,1), 1)
    node3 = Node((0,0.25), 0.5)

    assert not node1.is_in_collision(node2)
    assert not node2.is_in_collision(node1)
    assert node1.is_in_collision(node3)

def test_graph_collision_check():
    node1 = Node((0,0), 1)
    node2 = Node((1,1), 1)

    graph1 = GraphOn2DPlane([node1, node2], {0:[1], 1:[0]})

    node3 = Node((0.5,0.5), 1)

    graph2 = GraphOn2DPlane([node3], {})

    common_collision = CollisionChecker([graph1, graph2])

    assert common_collision.get_other_blocked_nodes(0,0) == [(1,0)]
    assert common_collision.get_other_blocked_nodes(0,1) == [(1,0)]
    assert common_collision.get_other_blocked_nodes(1,0) == [(0,0), (0,1)]

#test_node_collision_check()
#test_graph_collision_check()