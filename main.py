"""
Discrete Mathematics Project on the topic: 'Shortest path between cities'

Created by:
Olesia Arlamovska
Zlata-Antonina Leskiv
Mykhailo Pelyno
Zahar Kolodchak
Artem Yakymashchenko
"""

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

# def a_star(values: TODO, source: TODO, target: TODO) -> TODO:
#     """
#     TODO: Допиши опис
#     """
#     pass

# def greedy_algorithm(values: TODO, source: TODO, target: TODO) -> TODO:
#     """
#     TODO: Допиши опис
#     """
#     pass

if __name__ == "__main__":
    my_graph = load_data("Дніпро")[0]
    print("------------------")
    print(dijkstra_algorithm(my_graph, 13253800858, 11614899333))
