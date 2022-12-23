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
            nodedata['Longitude'] = ast.literal_eval(nodedata["position"])[0]
            nodedata['Latitude'] = ast.literal_eval(nodedata["position"])[1]

        elif "Longitude" in nodedata and "Latitude" in nodedata:
            with_lon = True
            pos[node] = [nodedata['Longitude'], nodedata['Latitude']]
        else:
            raise ValueError("Cannot determine node position.")

        if 'type' not in nodedata or nodedata['type'] != 'end_node':
            nodedata['type'] = 'repeater_node'

    
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
def draw_graph(G , center_nodes):
    pos = nx.get_node_attributes(G, 'pos')
    repeater_nodes = []
    end_nodes = []
    for node in G.nodes():
        if G.nodes[node]['type'] == 'repeater_node' or G.nodes[node]['type'] == 'new_repeater_node':
            repeater_nodes.append(node)
        else:
            end_nodes.append(node)
    fig, ax = plt.subplots(figsize=(7, 7))
    c_nodes = nx.draw_networkx_nodes(G=G, pos=pos, nodelist=center_nodes, node_shape='s', node_size=350,
                                       node_color=[[1.0, 120 / 255, 0.]], label="End Node", linewidths=3)
    c_nodes.set_edgecolor('k')
    rep_nodes = nx.draw_networkx_nodes(G=G, pos=pos, nodelist=repeater_nodes, node_size=350,
                                       node_color=[[1, 1, 1]], label="Repeater Node")
    rep_nodes.set_edgecolor('k')
    end_node_labels = {}
    repeater_node_labels = {}
    center_node_labels = {}
    for node, nodedata in G.nodes.items():
        # labels[node] = node
        if G.nodes[node]['type'] == 'end_node':  # or node in self.repeater_nodes_chosen:
            end_node_labels[node] = node
        else:
            repeater_node_labels[node] = node
    for node in center_nodes:
        center_node_labels[node] = node
    nx.draw_networkx_labels(G=G, pos=pos, labels=center_node_labels, font_size=7, font_weight="bold", font_color="w",
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
    print('shortest_path_dict len' , len(shortest_path_dict))
def add_quantum_repeater( G , L_max):
    print("====================== number of nodes 1 " , G.number_of_nodes() , " ===================================")

    q_node  = 0
    q_node_list = []
    q_node_edges = []
    pos = nx.get_node_attributes(G, 'pos')
    done_dest_node = {}
    removable_edge = []
       
    for i, j in G.edges():

        length = G[i][j]['length']
            
        if length > L_max :
            lat1 = G.nodes[i]['Latitude']
            lon1 = G.nodes[i]['Longitude']
            lat2 = G.nodes[j]['Latitude']
            lon2 = G.nodes[j]['Longitude']
            node1 = i
            for it in range(1 ,  int(length / L_max) + 1):
                node_data = {}
                dist = it * L_max
                lat3 , lon3 = get_intermediate_point(lat1 , lon1 , lat2 , lon2 , dist)
                print(i , it ,"QN", q_node , lon3 , lat3   , dist)
                print("calculated distance:" , get_distance_long_lat(lat1 , lon1 , lat3 , lon3))
                node2 = "QN" +str(q_node) 
                node_data['node'] = node2
                node_data['Latitude'] = float(lat3)
                node_data['Longitude'] = float(lon3)

                # if q_node == 0:
                #     print('42222:', i , j , lat1 , lon1 , lat2 , lon2 , length , L_max , dist)



                q_node_list.append(node_data)
                q_node_edges.append((node1 , node2))
                node1 = node2
                q_node += 1
            q_node_edges.append((node2 , j))
            done_dest_node[j] = 1
            removable_edge.append((i, j))
            # print('== ' , i , '-' , j)
    

    for node_data in q_node_list:
        G.add_node(node_data['node'], Longitude=node_data['Longitude'] , Latitude=node_data['Latitude'])
        G.nodes[node_data['node']]['type'] = 'new_repeater_node'
        pos[node_data['node']] = [node_data['Longitude'], node_data['Latitude']]
    for i , j in removable_edge:
        # print(i , '-' , j)
        G.remove_edge(i , j)

    G.add_edges_from(q_node_edges)
    for i, j in G.edges():
        if 'length' not in G[i][j]:
                _compute_dist_lat_lon(G)
    nx.set_node_attributes(G, pos, name='pos')
    print("====================== number of nodes 2 " , G.number_of_nodes() , " ===================================")
    # draw_graph(G , [])
def get_intermediate_point(lat1 , lon1 , lat2 , lon2 , d):
    constant = np.pi / 180
    R = 6371
    # φ1 = lat1 * constant
    # λ1 = lon1 * constant
    # φ2 = lat2 * constant
    # λ2 = lon2 * constant

    φ1 = np.radians(lat1) 
    λ1 = np.radians(lon1)  
    φ2 = np.radians(lat2)  
    λ2 = np.radians(lon2)
    y = np.sin(λ2-λ1) * np.cos(φ2)
    x = np.cos(φ1)*np.sin(φ2) -  np.sin(φ1)*np.cos(φ2)*np.cos(λ2-λ1)
    θ = np.arctan2(y, x)
    brng = θ
    # brng = (θ*180/np.pi + 360) % 360;  #in degrees
    # brng = brng * constant

    φ3 = np.arcsin( np.sin(φ1)*np.cos(d/R ) + np.cos(φ1)*np.sin(d/R )*np.cos(brng) )
    λ3 = λ1 + np.arctan2(np.sin(brng)*np.sin(d/R )*np.cos(φ1),  np.cos(d/R )-np.sin(φ1)*np.sin(φ2))

    return np.round(φ3/constant , 2) , np.round(λ3/constant , 2)


def get_distance_long_lat(lat1 , lon1 , lat2 , lon2):
    R = 6371   #in km 
    lon1 = np.radians(lon1)
    lon2 = np.radians(lon2)
    lat1 = np.radians(lat1)
    lat2 = np.radians(lat2)
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    a = np.sin(delta_lat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * (np.sin(delta_lon / 2) ** 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    dist = np.round(R * c, 5)
    return dist