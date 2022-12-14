import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import networkx as nx
import itertools
import ast

shortest_path_dict = {}

def read_graph_from_gml(file):

    G = nx.read_gml(file)


    pos = {}
    with_lon = False
    for node, nodedata in G.nodes.items():
        if "position" in nodedata:
            pos[node] = ast.literal_eval(nodedata["position"])
        elif "Longitude" in nodedata and "Latitude" in nodedata:
            with_lon = True
            pos[node] = [nodedata['Longitude'], nodedata['Latitude']]
        else:
            raise ValueError("Cannot determine node position.")

        if 'type' not in nodedata or nodedata['type'] != 'end_node':
            nodedata['type'] = 'repeater_node'

    
    for i, j in G.edges():
        if 'length' not in G[i][j]:
            if with_lon:
                _compute_dist_lat_lon(G)

    nx.set_node_attributes(G, pos, name='pos')
 
    return G

def _compute_dist_lat_lon(graph):
    """Compute the distance in km between two points based on their latitude and longitude.
        Assumes both are given in radians."""
    R = 6371  # Radius of the earth in km
    for edge in graph.edges():
        node1, node2 = edge
        lon1 = np.radians(graph.nodes[node1]['Longitude'])
        lon2 = np.radians(graph.nodes[node2]['Longitude'])
        lat1 = np.radians(graph.nodes[node1]['Latitude'])
        lat2 = np.radians(graph.nodes[node2]['Latitude'])
        delta_lat = lat2 - lat1
        delta_lon = lon2 - lon1
        a = np.sin(delta_lat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * (np.sin(delta_lon / 2) ** 2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
        dist = np.round(R * c, 5)
        graph.edges[node1, node2]['length'] = dist
def draw_graph(G):
    pos = nx.get_node_attributes(G, 'pos')
    repeater_nodes = []
    end_nodes = []
    for node in G.nodes():
        if G.nodes[node]['type'] == 'repeater_node' or G.nodes[node]['type'] == 'new_repeater_node':
            repeater_nodes.append(node)
        else:
            end_nodes.append(node)
    fig, ax = plt.subplots(figsize=(7, 7))
    end_nodes = nx.draw_networkx_nodes(G=G, pos=pos, nodelist=end_nodes, node_shape='s', node_size=150,
                                       node_color=[[1.0, 120 / 255, 0.]], label="End Node", linewidths=3)
    end_nodes.set_edgecolor('k')
    rep_nodes = nx.draw_networkx_nodes(G=G, pos=pos, nodelist=repeater_nodes, node_size=150,
                                       node_color=[[1, 1, 1]], label="Repeater Node")
    rep_nodes.set_edgecolor('k')
    end_node_labels = {}
    repeater_node_labels = {}
    for node, nodedata in G.nodes.items():
        # labels[node] = node
        if G.nodes[node]['type'] == 'end_node':  # or node in self.repeater_nodes_chosen:
            end_node_labels[node] = node
        else:
            repeater_node_labels[node] = node
    nx.draw_networkx_labels(G=G, pos=pos, labels=end_node_labels, font_size=7, font_weight="bold", font_color="w",
                            font_family='serif')
    nx.draw_networkx_labels(G=G, pos=pos, labels=repeater_node_labels, font_size=5, font_weight="bold")
    nx.draw_networkx_edges(G=G, pos=pos, width=1)
    plt.axis('off')
    margin = 0.33
    fig.subplots_adjust(margin, margin, 1. - margin, 1. - margin)
    ax.axis('equal')
    fig.tight_layout()
    plt.show()

def compute_shortest_path(G):
    nodes = G.nodes()
    global shortest_path_dict 
    for i in nodes:
        for j in nodes:
            if i != j:
                # Use NetworkX to generate the shortest path with Dijkstra's algorithm
                (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
                # Store the path cost and the shortest path itself as a tuple in the dictionary
                shortest_path_dict[(i, j)] = (path_cost, sp)