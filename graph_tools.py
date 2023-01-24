import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import networkx as nx
import itertools
import ast
import math

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
def get_distance_between_nodes(G , node1 , node2):
    R = 6371  # Radius of the earth in km
    lon1 = np.radians(graph.nodes[node1]['Longitude'])
    lon2 = np.radians(graph.nodes[node2]['Longitude'])
    lat1 = np.radians(graph.nodes[node1]['Latitude'])
    lat2 = np.radians(graph.nodes[node2]['Latitude'])
    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1
    a = np.sin(delta_lat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * (np.sin(delta_lon / 2) ** 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    dist = np.round(R * c, 5)
    return dist
    

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
    print('in compute shortest path len' , len(nodes))
    global shortest_path_dict 
    for i in nodes:
        for j in nodes:
            if i != j:
                
                # Use NetworkX to generate the shortest path with Dijkstra's algorithm
                (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
                # Store the path cost and the shortest path itself as a tuple in the dictionary
                shortest_path_dict[(i, j)] = (path_cost, sp)
    print('shortest_path_dict len' , len(shortest_path_dict))



def compute_mst_centers(G , center_nodes):
    H = G.copy()
    for node in G.nodes():
        if ('type' in G.nodes[node] and  G.nodes[node]['type'] == "new_repeater_node") or node not in center_nodes:
            H.remove_node(node)
    for i in H.nodes():
        for j in H.nodes():
            if i != j:
                (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
                H.add_edge(i , j , length=path_cost)
                # print('path cost ' , i , j , path_cost)
    
    T = nx.minimum_spanning_tree(H , weight='length')
    # draw_graph(T , [])
    return T

def compute_edges_to_place_new_repeaters(G , center_nodes):
    T = compute_mst_centers(G , center_nodes)
    edge_list = list()
    for i , j in T.edges():
        (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
        for i in range(len(sp)):
            if i +1 < len(sp):
                edge_list.append((sp[i] , sp[i+1]))
    for node in G.nodes():
        if G.nodes[node]['type'] == 'end_node':
            # print('+++++++++= ' , G.edges(node))
            edge_list.append(list(G.edges(node))[0])
    
    return edge_list
def compute_edges_to_choose_more_centers(G , center_nodes):
    T = compute_mst_centers(G , center_nodes)
    edge_list = list()
    for i , j in T.edges():
        edge_list.append((i , j))
    for node in G.nodes():
        if G.nodes[node]['type'] == 'end_node':
            # print('+++++++++= ' , G.edges(node))
            edge_list.append(list(G.edges(node))[0])
    
    return edge_list

def add_quantum_repeater_between_centers( G , center_nodes , L_max):
    print("====================== number of nodes 1 " , G.number_of_nodes() , " ===================================")
    # print("type: " , G.nodes["SNLCA"]['type'])
    edge_list = compute_edges_to_place_new_repeaters(G , center_nodes)

    q_node  = 0
    q_node_list = []
    q_node_edges = []
    pos = nx.get_node_attributes(G, 'pos')
    done_dest_node = {}
    removable_edge = []
       
    for i, j in edge_list:
        length1 = 0
        length2 = 0
        center1 = None
        center2 = None
        q_node_added = False
        if i not in center_nodes:
            center1 = get_nearest_center(G , i , center_nodes , L_max)
            if center1 != j:
                length1 = get_distance(i , center1)
        if j not in center_nodes:
            center2 = get_nearest_center(G, j , center_nodes , L_max)
            if center2 != i:
                length2 = get_distance(j , center2)

        if center1 != None and center1 == center2:
            continue
        
        

        length = G[i][j]['length']

            
        if length > L_max :
            length = length + length1 + length2
            lat1 = G.nodes[i]['Latitude']
            lon1 = G.nodes[i]['Longitude']
            lat2 = G.nodes[j]['Latitude']
            lon2 = G.nodes[j]['Longitude']
            lat3 = lat1
            lon3 = lon1
            node1 = i
            node2 = i
            placement_dist = length / (int(length / L_max) + 1)
            for it in range(1 ,  int(length / L_max) + 1):

                node_data = {}
                dist = it * placement_dist
                if it == 1:
                    if dist <= get_distance(center1 , i):
                        center_nodes.add(i)
                        # continue
                    # dist = dist - length1

                if  placement_dist >= get_distance_long_lat(lat3 , lon3 , lat2 , lon2):
                    node2 = node1
                    center_nodes.add(j)
                    continue
                lat3 , lon3 = get_intermediate_point(lat1 , lon1 , lat2 , lon2 , dist)
                # print(lat1 , lon1 , lat3 , lon3)
                # if i == 'SNLCA' or j == 'SNLCA':
                #     print(i , j , placement_dist , length1 , dist , center1 , center2 , get_distance(center1 , i) , G[i][j]['length'])

                node2 = "QN" +str(q_node) 
                node_data['node'] = node2
                node_data['Latitude'] = float(lat3)
                node_data['Longitude'] = float(lon3)


                q_node_list.append(node_data)
                q_node_edges.append((node1 , node2))
                node1 = node2
                q_node += 1
                q_node_added = True
            if q_node_added > 0:
                q_node_edges.append((node2 , j))
                removable_edge.append((i, j))
            # print('== ' , i , '-' , j)
    

    for node_data in q_node_list:
        G.add_node(node_data['node'], Longitude=node_data['Longitude'] , Latitude=node_data['Latitude'])
        G.nodes[node_data['node']]['type'] = 'new_repeater_node'
        pos[node_data['node']] = [node_data['Longitude'], node_data['Latitude']]
        # add new nodes as center nodes
        center_nodes.add(node_data['node'])
    for i , j in removable_edge:
        # print(i , '-' , j)
        G.remove_edge(i , j)

    G.add_edges_from(q_node_edges)
    for i, j in G.edges():
        if 'length' not in G[i][j]:
                _compute_dist_lat_lon(G)
    nx.set_node_attributes(G, pos, name='pos')
    print("====================== number of nodes 2 " , G.number_of_nodes() , " ===================================")
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
    brng2 = (θ*180/np.pi + 360) % 360;  #in degrees


    φ3 = np.arcsin( np.sin(φ1)*np.cos(d/R ) + np.cos(φ1)*np.sin(d/R )*np.cos(brng) )
    λ3 = λ1 + np.arctan2(np.sin(brng)*np.sin(d/R )*np.cos(φ1),  np.cos(d/R )-np.sin(φ1)*np.sin(φ2))

    # φ3 = np.arcsin( np.sin(φ1)*np.cos(np.radians(d/R) ) + np.cos(φ1)*np.sin(np.radians(d/R) )*np.cos(brng) )
    # λ3 = λ1 + np.arctan2(np.sin(brng)*np.sin(np.radians(d/R) )*np.cos(φ1),  np.cos(np.radians(d/R) )-np.sin(φ1)*np.sin(φ2))

    return np.round(np.degrees(φ3) , 14) , np.round(np.degrees(λ3) , 14)






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
def get_distance(node1 , node2 ):
    dist = -1
    if (node1 , node2) in shortest_path_dict:
        dist = shortest_path_dict[(node1 , node2)][0]
    elif (node2 , node1) in shortest_path_dict:
        dist = shortest_path_dict[(node2 , node1)][0]

    return dist

def get_nearest_center(G , node , center_nodes , L_max):
    for i in range(1 , 10):
        length = nx.single_source_shortest_path_length(G ,source=node, cutoff=i)
        for n in length:
            if n!=node:
                if n in center_nodes and get_distance(node , n) <= L_max:
                    return n
    return None