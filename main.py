"""
Discrete Mathematics Project on the topic: 'Shortest path between cities'

Created by:
Olesia Arlamovska
Zlata-Antonina Leskiv
Mykhailo Pelyno
Zahar Kolodchak
Artem Yakymashchenko
"""
import math
import osmnx as ox

def load_data(place_name: str) -> tuple:
    """
    Loads data for the graph

    Uses OSMnx module to load the data

    :param place_name: Name of the place for which loads data
    :return: Tuple with 2 graph:
    1st one for search algorithms;
    2nd one for visualisation
    """
    print(f"Loading map: {place_name}")

    # 1. Load the road graph ('drive' is for car)
    graph_map = ox.graph_from_place(place_name, network_type='drive')

    # Format: { node_id: { neighbor_id: length } }
    graph_dict = {}

    # 2. Make keys for all nodes
    for node in graph_map.nodes():
        graph_dict[node] = {}

    # 3. u = from where, v = to where, data = info about the road
    for u, v, data in graph_map.edges(data=True):
        distance = data.get('length', 1) # Take the length in meters
        graph_dict[u][v] = distance

    print(f"Map loading successful: {place_name}")

    return (graph_dict, graph_map)  # Variable 'graph_map' will be needed for visualization


def get_coordinates(graph: dict, node_id: int) -> tuple:
    """
    Get node's coordinates

    :param graph: Graph from which we get coordinates
    :param node_id: id of the node which coordinates we are finding
    :return: Tuple with coordinates (x, y)
    """
    node = graph.nodes[node_id]
    return (node['x'], node['y'])

#~~~~~~~~~~~~DIJKSTRA~~~~~~~~~~~~~~~~~
def dijkstra_algorithm(graph: dict, source: int, target: int) -> tuple:
    """
    Finds the shortest path from source to target

    Function uses Dijkstra's algorithm to find the shortest path

    :param graph: Graph (as adjacency list) on which we find this path
    :param source: Source node from which we start
    :param target: Target node which we look for
    :return: Tuple with the shortest path from source to node and weight of this path
    """
    queue = set()
    dist = {}
    path = {}

    for node in graph:
        dist[node] = float("inf")
        path[node] = [source]
        queue.add(node)

    dist[source] = 0

    while queue and target in queue: # Do while queue is not empty and target was not visited
        min_vert = float('inf')
        for node in queue:
            if min_vert == float("inf"): # Check if min_vert was not defined beforehand
                min_vert = node

            if dist[node] < dist[min_vert]:
                min_vert = node

        node = min_vert
        queue.remove(node)

        for u in graph[node]: # Check node's neighbours
            alt = dist[node] + graph[node][u]

            if alt < dist[u]:
                dist[u] = alt
                path[u] = path[node][:]
                path[u].append(u)

    return (path[target], dist[target])

#~~~~~~~~~~~~GREEDY~~~~~~~~~~~~~~~~~
# def revert_to_km_on_earth(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
#     """
#     Docstring for revert_to_km

#     :param lon1: longtitude of point 1
#     :type lon1: float
#     :param lat1: latitude of point 1
#     :type lat1: float
#     :param lon2: longtitude of point 2
#     :type lon2: float
#     :param lat2: latitude of point 2
#     :type lat2: float
#     :return: Description
#     :rtype: float
#     """
#     pass

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
    graph_dict, graph = values
    current_node = start
    visited = {start}
    path = [start]
    final_length = 0.0
    while current_node != goal:
        neighbours = graph_dict.get(current_node, {})
        candidates = [n for n in neighbours if n not in visited]
        if not candidates:
            return None, None

        goal_x, goal_y = get_coordinates(graph, goal)
        next_node = min(candidates,
key=lambda n: distance_between_points(*get_coordinates(graph, n), goal_x, goal_y))
        #бере той елемент, у якого мінімальне значення distance

        final_length += graph_dict[current_node][next_node]
        path.append(next_node)
        visited.add(next_node)
        current_node = next_node
    return (path, final_length / 1000)

# def a_star(values: TODO, source: TODO, target: TODO) -> TODO:
#     """
#     TODO: Допиши опис
#     """
#     pass


if __name__ == "__main__":
    my_graph = load_data("Дніпро")[0]
    print("------------------")
    print("Dijkstra result:")
    print(dijkstra_algorithm(my_graph, 13253800858, 11614899333))
    print("------------------")
    print("Greedy result:")
    print(greedy_algorithm(load_data("Дніпро"), 13253800858, 11614899333))
    print("------------------")
    print("A* result:")
    
