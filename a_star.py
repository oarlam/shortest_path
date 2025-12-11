"""A* algorithm"""

import math

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
