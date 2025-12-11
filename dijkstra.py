"""Dijkstra Algorithm"""

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
