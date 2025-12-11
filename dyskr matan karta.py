import math
import osmnx as ox

import matplotlib.pyplot as plt


def load_data(place_name: str):
    """Завантажує граф доріг (тільки основні highways)."""
    print(f"Load map: {place_name}...")
    custom_filter = '["highway"~"motorway|trunk|primary|secondary|tertiary"]'
    G = ox.graph_from_place(place_name, network_type='drive', simplify=True, custom_filter=custom_filter)
    graph_dict = {node: {} for node in G.nodes()}
    for u, v, data in G.edges(data=True):
        graph_dict[u][v] = data.get('length', 1)
    print("Done loading.")
    return graph_dict, G

def get_coordinates(graph, node_id):
    node = graph.nodes[node_id]
    return node['x'], node['y']


def count_h_value(G, node_id, end_id):
    node = G.nodes[node_id]
    end = G.nodes[end_id]
    return math.sqrt((node['x'] - end['x'])**2 + (node['y'] - end['y'])**2)

def a_star_method(G, graph, start_id, end_id):
    nodes_info = {start_id: {'g_value': 0, 'f_value': count_h_value(G, start_id, end_id), 'parent': None}}
    queue = [start_id]
    checked = set()

    while queue:
        queue.sort(key=lambda x: nodes_info[x]['f_value'])
        current_node = queue.pop(0)
        checked.add(current_node)
        if current_node == end_id:
            return nodes_info
        for node, distance in graph[current_node].items():
            if node in checked:
                continue
            g_value = nodes_info[current_node]['g_value'] + distance
            f_value = g_value + count_h_value(G, node, end_id)
            if node not in queue or nodes_info.get(node, {'f_value': float('inf')})['f_value'] > f_value:
                nodes_info[node] = {'g_value': g_value, 'f_value': f_value, 'parent': current_node}
                if node not in queue:
                    queue.append(node)
    return nodes_info

def get_path_for_astar(nodes_info, start_id, end_id):
    if end_id not in nodes_info:
        return None, None
    path = [end_id]
    node = end_id
    while node != start_id:
        node = nodes_info[node]['parent']
        path.append(node)
    path.reverse()
    distance = nodes_info[end_id]['g_value']
    return path, distance


def greedy_algorithm(values, start, goal):
    graph_dict, graph = values
    current_node = start
    visited = {start}
    path = [start]
    final_length = 0.0

    while current_node != goal:
        neighbours = [n for n in graph_dict.get(current_node, {}) if n not in visited]
        if not neighbours:
            return None, None
        goal_x, goal_y = get_coordinates(graph, goal)
        next_node = min(neighbours, key=lambda n: math.sqrt(
            (get_coordinates(graph, n)[0] - goal_x)**2 + (get_coordinates(graph, n)[1] - goal_y)**2
        ))
        final_length += graph_dict[current_node][next_node]
        path.append(next_node)
        visited.add(next_node)
        current_node = next_node

    return path, final_length

def dijkstra(graph: dict, source: int, target: int = None):
    """
    Повертає (dist, prev) де:
      - dist[v] = мінімальна відстань від source до v
      - prev[v] = попередній вузол на найкоротшому шляху до v (None для source)
    Якщо передано target, можна відновити шлях від source до target.
    """

    dist = {v: float('inf') for v in graph}
    prev = {v: None for v in graph}
    dist[source] = 0
    unvisited = set(graph.keys())

    while unvisited:

        u = min(unvisited, key=lambda v: dist[v])
        unvisited.remove(u)

        if target is not None and u == target:
            break
        for v, w in graph.get(u, {}).items():
            alt = dist[u] + w
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

    return dist, prev

def reconstruct_path(prev: dict, source: int, target: int):
    if prev.get(target) is None and source != target:
        return None
    path = []
    u = target
    while u is not None:
        path.append(u)
        if u == source:
            break
        u = prev[u]
    path.reverse()
    return path


def plot_multiple_paths(G, routes: dict):
    """
    G - osmnx graph
    routes - dict label -> path_list (or None)
    Малює граф (основні дороги) та накладає маршрути (різні кольори).
    """
    fig, ax = ox.plot_graph(G, show=False, close=False, node_size=5, edge_color='lightgray', edge_alpha=0.6)

    colors = {'A*':'red', 'Greedy':'orange', 'Dijkstra':'blue'}
    for label, path in routes.items():
        if not path:
            continue
        x_coords = [G.nodes[n]['x'] for n in path]
        y_coords = [G.nodes[n]['y'] for n in path]
        ax.plot(x_coords, y_coords, color=colors.get(label, 'black'), linewidth=3, alpha=0.8, label=label)

        ax.scatter(x_coords[0], y_coords[0], color='green', s=40, zorder=5)
        ax.scatter(x_coords[-1], y_coords[-1], color='magenta', s=40, zorder=5)

    ax.legend()
    plt.show()


if __name__ == "__main__":
    graph_dict, G = load_data("Vermont, USA")


    start_city = input("Start city: ")
    end_city   = input("End city: ")

    start_coords = ox.geocode(f"{start_city}, Vermont, USA")
    end_coords   = ox.geocode(f"{end_city}, Vermont, USA")
    start_id = ox.distance.nearest_nodes(G, start_coords[1], start_coords[0])
    end_id   = ox.distance.nearest_nodes(G, end_coords[1], end_coords[0])

    # A*
    nodes_info = a_star_method(G, graph_dict, start_id, end_id)
    path_a, dist_a = get_path_for_astar(nodes_info, start_id, end_id)
    if path_a:
        print(f"A* Path: {dist_a:.2f} m, nodes: {len(path_a)}")
    else:
        print("A* did not find a path.")

    # Greedy
    path_g, dist_g = greedy_algorithm((graph_dict, G), start_id, end_id)
    if path_g:
        print(f"Greedy Path: {dist_g/1000:.3f} km, nodes: {len(path_g)}")
    else:
        print("Greedy failed (got stuck).")

    # Dijkstra
    dist_d, prev = dijkstra(graph_dict, start_id, target=end_id)
    path_d = reconstruct_path(prev, start_id, end_id)
    if path_d:
        print(f"Dijkstra Path: {dist_d[end_id]:.2f} m, nodes: {len(path_d)}")
    else:
        print("Dijkstra did not find a path.")

    
    routes = {'A*': path_a, 'Greedy': path_g, 'Dijkstra': path_d}
    plot_multiple_paths(G, routes)
