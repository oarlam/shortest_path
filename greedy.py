"""Greedy Algorithm"""

import math
import osmnx as ox

def load_data(path: str):
    """
    Loads a road network graph from a file and converts it into a dictionary representation

    :param path: Path to the file from which we load the map
    """
    print(f"Loading map: {path} ...")

    graph = ox.graph_from_xml(path)
    graph_dict = {node: {} for node in graph.nodes()}

    for u, v, data in graph.edges(data=True):
        graph_dict[u][v] = data.get('length', 1)

    print("Done loading.")

    print(graph_dict)

    return graph_dict, graph

def get_coordinates(graph: dict, node_id: int) -> tuple:
    """
    Get node's coordinates

    :param graph: Graph from which we get coordinates
    :param node_id: id of the node which coordinates we are finding
    :return: Tuple with coordinates (x, y)
    """
    node = graph.nodes[node_id]
    return (node['x'], node['y'])

def distance_between_points(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    Docstring for distance_between_points
    Calculates distance between points with formula:
    d=sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    It's euclidean formula btw
​
    :param x1: x-coordinate of 1 node
    :type x1: float
    :param y1: y-coordinate of 1 node
    :type y1: float
    :param x2: x-coordinate of 2 node
    :type x2: float
    :param y2: y-coordinate of 2 node
    :type y2: float
    :return: final distance between points
    :rtype: float
    """
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def greedy_algorithm(values: tuple, start: int, goal: int) -> tuple:
    """"
    Greedy Algorythm😛😛😛:
    на кожному кроці обираємо сусіда, який
    НАЙБЛИЖЧИЙ до final Destination по прямій (за координатами).

    At each step the algorithm chooses the neighbouring node that is
    closest to the destination in a straight line, based on OpenStreetMap
    coordinates. The algorithm makes a locally optimal decision at every
    step and does not guarantee the globally shortest path.

    The map is treated as a FLAT surface for simplicity, and distances
    between points are computed using the Euclidean formula.

    :param values: Tuple (graph_dict, graph) returned by load_data()
    :param start: ID of the starting node
    :param goal: ID of the destination node
    :return: A tuple (path, distance), where:
             - path is a list of node IDs from start to goal,
             - distance is the total length of the found path.
             If the algorithm gets stuck, (None, None) is returned.
    """
    # previous = start
    graph_dict, graph = values

    current_node = start
    path = [start]
    visited = {start}
    final_length = []
    dead_ends = set()

    goal_x, goal_y = get_coordinates(graph, goal)

    while current_node != goal:
        neighbours = graph_dict.get(current_node, {})

        candidates = [
            n for n in neighbours
            if n not in visited and n not in dead_ends
        ]

        if not candidates:
            dead_ends.add(current_node)

            if len(path) == 1:
                return None, None

            path.pop()
            if final_length:
                final_length.pop()
            current_node = path[-1]
            continue
        next_node = min(
            candidates,
            key=lambda n: distance_between_points(
                *get_coordinates(graph, n),
                goal_x,
                goal_y
            )
        )

        final_length.append(neighbours[next_node])
        path.append(next_node)
        visited.add(next_node)
        current_node = next_node

    return path, sum(final_length)
