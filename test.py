# import osmnx as ox
# import matplotlib.pyplot as plt

# # Define the place and network type
# place = "Paris, France"
# network_type = "drive"

# # Download the street network
# G = ox.graph_from_place(place, network_type=network_type)

# # Plot the graph
# fig, ax = ox.plot_graph(G)
# plt.show()
# print(fig)
# print(ax)

import osmnx as ox
import networkx as nx

# # Get the two ends of a street segment for a starting point
# orig_node = list(G.nodes)[0]

# # Get a destination node further down the street
# dest_node = list(G.nodes)[50]

# # Find the shortest path
# route = nx.shortest_path(G, orig_node, dest_node, weight="length")

# # Visualize the route
# fig, ax = ox.plot_graph_route(G, route, node_size=0, bgcolor='k')
