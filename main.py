"""
Discrete Mathematics Project on the topic: 'Shortest path between cities'

Created by:
Olesia Arlamovska
Zlata-Antonina Leskiv
Mykhailo Pelyno
Zahar Kolodchak
Artem Yakymashchenko
"""

import time
import os
import sys
import argparse
import osmnx as ox
import matplotlib.pyplot as plt
import a_star
import greedy
import dijkstra

parser = argparse.ArgumentParser(description="Finds and visualizes shortest path between to cities")
parser.add_argument("file", type=str, help="Path to file from which we load map")
parser.add_argument("source", type=int, help="Source node id from which we start the search")
parser.add_argument("target", type=int, help="Target node id to which we go")

args = parser.parse_args()
file = args.file
source = args.source
target = args.target



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

    return graph_dict, graph


def plot_multiple_paths(graph, routes: dict):
    """
    Draws a graph (main roads) and overlays routes (different colors).

    :param graph: An osmnx graph
    :param routes: Routes which we overlay
    """
    fig, ax = ox.plot_graph(graph, show=False, close=False, \
                            node_size=5, edge_color='lightgray', edge_alpha=0.6)

    colors = {'A*':'red', 'Greedy':'orange', 'Dijkstra':'blue'}
    for label, path in routes.items():
        if not path:
            continue
        x_coords = [graph.nodes[n]['x'] for n in path]
        y_coords = [graph.nodes[n]['y'] for n in path]
        ax.plot(x_coords, y_coords, color=colors.get(label, 'black'), \
                linewidth=3, alpha=0.8, label=label)

        ax.scatter(x_coords[0], y_coords[0], color='green', s=40, zorder=5)
        ax.scatter(x_coords[-1], y_coords[-1], color='magenta', s=40, zorder=5)

    ax.legend()
    plt.show()


def main(path: str, start_id: int, end_id: int):
    """
    Main function for execution

    :param path: Path to the file from which we load map
    :param start_id: ID of the search start
    :param end_id: ID for which we are looking for
    """
    graph_dict, graph = load_data(path)

    # A*
    time_a = time.time()
    nodes_info = a_star.a_star_method(graph, graph_dict, start_id, end_id)
    path_a, dist_a = a_star.get_path_for_astar(nodes_info, start_id, end_id)
    time_a = time.time() - time_a
    if path_a:
        print(f"A* Path: {dist_a/1000:.3f} km, nodes: {len(path_a)}, time: {time_a:.3f} s")
    else:
        print("A* did not find a path.")

    # Greedy
    try:
        time_g = time.time()
        path_g, dist_g = greedy.greedy_algorithm((graph_dict, graph), start_id, end_id)
        time_g = time.time() - time_g
        if path_g:
            print(f"Greedy Path: {dist_g/1000:.3f} km, nodes: {len(path_g)}, time: {time_g:.3f} s")
        else:
            print("Greedy failed (got stuck).")
    except IndexError:
        path_g = []
        print("Greedy failed")

    # Dijkstra

    time_d = time.time()
    path_d, dist_d = dijkstra.dijkstra_algorithm(graph_dict,start_id,end_id)
    time_d = time.time() - time_d
    if path_d:
        print(f"Dijkstra Path: {dist_d/1000:.3f} km, nodes: {len(path_d)}, time: {time_d:.3f} s")
    else:
        print("Dijkstra did not find a path.")

    routes = {'A*': path_a, 'Greedy': path_g, 'Dijkstra': path_d}
    plot_multiple_paths(graph, routes)

if not os.path.exists(file):
    print(f"destination path '{file}' does not exist")
    sys.exit(1)

if os.path.isdir(file):
    print(f"destination path '{file}' refers to directory and not file")
    sys.exit(1)

if not file.endswith(".osm"):
    print(f"'{file}' should be an .osm file")
    sys.exit(1)

try:
    main(file, source, target)
except KeyError:
    print("ID was not found, please write correct one")
    sys.exit(1)
