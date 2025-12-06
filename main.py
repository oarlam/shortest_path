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

#~~~~~~~~~~~~A~STAR~~~~~~~~~~~~~~~~~
def count_h_value(G, node_id: int, end_id: int) -> float:
    '''
    Calculates the Euclidean distance between the
    current node and the end node on a planar projected graph.

    G: The projected graph.
    node_id: The unique ID of the current node.
    end_id: The unique ID of the goal node.

    Returns: float: The straight-line distance between the two nodes.
    '''
    node = G.nodes[node_id]
    end = G.nodes[end_id]
    return math.sqrt((node['x'] - node['y']) ** 2 + (end['x'] - end['y']) ** 2)


def a_star_method(G, graph: dict, start_id: int, end_id: int) -> dict[dict]:
    """
    Performs an A-star algoritm and returns all required data to build a path

    Function uses A-star algorithm to find the shortest path

    :param G: Graph created with osmnx library, on which we find this path
    :param graph: Dictionary with all the nodes
    :param start_id: Source node from which we start
    :param end_id: Target node which we look for
    :return: Dictionary with distances and parents
    """
    # Format:  {node_id : {g_value: , f_value: , parent: }}
    nodes_info = {start_id: {'g_value' : 0, 'f_value' : 0, 'parent': None}}
    queque = [start_id]
    checked = set()

    while len(queque) > 0:
        queque.sort(key = lambda x: nodes_info[x]['f_value'])
        current_node = queque.pop(0)
        checked.add(current_node)
        if current_node == end_id:
            return nodes_info
        neighbours = graph[current_node]
        for node in neighbours:
            if node in checked:
                continue
            distance = neighbours[node]
            g_value = nodes_info[current_node]['g_value'] + distance
            f_value = g_value + count_h_value(G, start_id, end_id)
            if node not in queque or \
            (node in queque and nodes_info[node]['f_value'] > f_value):
                nodes_info[node] = {'g_value': g_value, 'f_value': f_value, 'parent': current_node}
                if node not in queque:
                    queque += [node]

def get_path_for_astar(nodes_info: dict, start_id: int, end_id: int) -> tuple[list, float]:
    '''
    Reconstructs the shortest path found by the A* search algorithm and
    returns its total length.

    The function works by backtracking from the goal node to the start node
    using the 'parent' pointers stored during the search.
    Parameters:
    nodes_info (dict): A dictionary containing the 'g_value' and 'parent' of each node.
    start_id (int): ID of the starting node.
    end_id (int): ID of the goal node.

    Returns:
    A tuple containing:
        list: The sequence of node IDs representing the shortest path,
            starting from 'start_id' and ending at 'end_id'.
        float: The lenght of the shortest path from the start to the end node.
    '''
    distance = nodes_info[end_id]['g_value']
    path = [end_id]
    node = end_id
    while True:
        node = nodes_info[node]['parent']
        path += [node]
        if node == start_id:
            return path[::-1], distance

if __name__ == "__main__":
    my_graph, osmnx_graph = load_data("Дніпро")
    print("------------------")
    print("Dijkstra result:")
    print(dijkstra_algorithm(my_graph, 13253800858, 11614899333))
    print("------------------")
    print("Greedy result:")
    print(greedy_algorithm(my_graph, osmnx_graph, 13253800858, 11614899333))
    print("------------------")
    print("A* result:")
    steps = greedy_algorithm(my_graph, osmnx_graph, 13253800858, 11614899333)
    print(get_path_for_astar(steps, 13253800858, 11614899333))




