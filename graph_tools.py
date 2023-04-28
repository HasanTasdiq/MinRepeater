import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import networkx as nx
import itertools
import ast
import math

shortest_path_dict = {}

class nodeTree:
    def __init__(self, node , parent):
        self.node = node
        self.parent = parent
    node = None
    children = None
    parent = None


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
    find_end_nodes(G)
    
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
        # graph.edges[node1, node2]['weight'] = dist

def _compute_dist_cartesian(graph):
    """Compute the distance in km between two points based on their Cartesian coordinates."""
    for edge in graph.edges():
        node1, node2 = edge
        dx = np.abs(graph.nodes[node1]['xcoord'] - graph.nodes[node2]['xcoord'])
        dy = np.abs(graph.nodes[node1]['ycoord'] - graph.nodes[node2]['ycoord'])
        dist = np.round(np.sqrt(np.square(dx) + np.square(dy)), 5)
        graph.edges[node1, node2]['length'] = dist
def draw_graph(G , center_nodes=[]):
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
                                       node_color=[[0, 1, .6]], label="End Node", linewidths=3)
    c_nodes.set_edgecolor('k')

    for c in center_nodes:
        repeater_nodes.remove(c)
    rep_nodes = nx.draw_networkx_nodes(G=G, pos=pos, nodelist=repeater_nodes, node_size=350,
                                       node_color=[[255/255, 195/255, 0/255]], label="Repeater Node")
    rep_nodes.set_edgecolor('k')
    end_node_labels = {}
    repeater_node_labels = {}
    center_node_labels = {}
    for node, nodedata in G.nodes.items():
        # labels[node] = node
        # if G.nodes[node]['type'] == 'end_node':  # or node in self.repeater_nodes_chosen:
        #     end_node_labels[node] = node
        # else:
        #     repeater_node_labels[node] = node
        repeater_node_labels[node] = node

    for node in center_nodes:
        center_node_labels[node] = node
    nx.draw_networkx_labels(G=G, pos=pos, labels=center_node_labels, font_size=9, font_weight="bold", font_color="w",
                            font_family='serif')
    nx.draw_networkx_labels(G=G, pos=pos, labels=repeater_node_labels, font_size=9, font_weight="bold")
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

def create_graph_on_unit_cube(n_repeaters, radius, draw, seed=2):
    """Create a geometric graph where nodes randomly get assigned a position. Two nodes are connected if their distance
    does not exceed the given radius."""
    np.random.seed = seed
    G = nx.random_geometric_graph(n=n_repeaters, radius=radius, dim=2, seed=seed)
    for node in G.nodes():
        G.nodes[node]['type'] = 'repeater_node'
    color_map = ['blue'] * len(G.nodes)
    # Create the end nodes
    G.add_node("C", pos=[0, 0], type='end_node')
    G.add_node("B", pos=[1, 1], type='end_node')
    G.add_node("A", pos=[0, 1], type='end_node')
    G.add_node("D", pos=[1, 0], type='end_node')
    G.nodes[3]['pos'] = [0.953, 0.750]
    G.nodes[5]['pos'] = [0.25, 0.50]
    # Manually connect the end nodes to the three nearest nodes
    G.add_edge("C", 8)
    G.add_edge("C", 5)
    G.add_edge("C", 2)
    G.add_edge("B", 9)
    G.add_edge("B", 4)
    G.add_edge("B", 3)
    G.add_edge("A", 1)
    G.add_edge("A", 2)
    G.add_edge("A", 9)
    G.add_edge("D", 3)
    G.add_edge("D", 6)
    G.add_edge("D", 7)
    color_map.extend(['green'] * 4)
    for node in G.nodes():
        G.nodes[node]['xcoord'] = G.nodes[node]['pos'][0]
        G.nodes[node]['ycoord'] = G.nodes[node]['pos'][1]
    # Convert node labels to strings
    label_remapping = {key: str(key) for key in G.nodes() if type(key) is not str}
    G = nx.relabel_nodes(G, label_remapping)
    if draw:
        draw_graph(G)
    _compute_dist_cartesian(G)
    return G

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

def compute_edges_to_place_new_repeaters_k(G , center_nodes):
    T = compute_mst_centers(G , center_nodes)
    edge_list = list()
    for node1 , node2 in T.edges():
        (path_cost, sp) = nx.single_source_dijkstra(G=G, source=node1, target=node2, weight='length')
        for k in range(len(sp)):
            if k +1 < len(sp):
                edge_list.append((sp[k] , sp[k+1]))
    for node in G.nodes():
        if G.nodes[node]['type'] == 'end_node':
            # print('+++++++++= ' , G.edges(node))
            edge_list.append(list(G.edges(node))[0])
    
    return edge_list

def compute_edges_to_place_new_repeaters(G , center_nodes , k):
    edge_list = set()

    if k ==1:
        edge_list.update(compute_edges_to_place_new_repeaters_k(G , center_nodes))
    else:
        random_down_nodes_ = list(itertools.combinations(center_nodes, r=k-1))
        for tup in random_down_nodes_:
            G2 = G.copy()
            for i in range(k-1):
                G2.remove_node(tup[i])
            edge_list.update(compute_edges_to_place_new_repeaters_k(G2 , center_nodes))
    return edge_list


def compute_edges_to_choose_more_centers(G , center_nodes , Lmax , k):
    edge_list = set()

    if k ==1:
        edge_list.update(compute_edges_to_choose_more_centers_k(G , center_nodes , Lmax))
    else:
        random_down_nodes_ = list(itertools.combinations(center_nodes, r=k-1))
        for tup in random_down_nodes_:
            G2 = G.copy()
            for i in range(k-1):
                G2.remove_node(tup[i])
            edge_list.update(compute_edges_to_choose_more_centers_k(G2 , center_nodes , Lmax))
    return edge_list
    
def compute_edges_to_choose_more_centers_k(G , center_nodes , Lmax):
    T = compute_mst_centers(G , center_nodes)
    edge_list = list()
    for i , j in T.edges():
        if get_distance(i , j ) > Lmax:
            edge_list.append((i , j))
    # for node in G.nodes():
    #     if G.nodes[node]['type'] == 'end_node':
    #         edge_list.append(list(G.edges(node))[0])
    
    return edge_list

def add_quantum_repeater_between_centers( G , center_nodes , mandatory_centers , L_max , k):
    print("====================== number of nodes 1 " , G.number_of_nodes() , " ===================================")
    # print("type: " , G.nodes["SNLCA"]['type'])
    edge_list = compute_edges_to_place_new_repeaters(G , center_nodes , k)
    print("edge list len " , len(edge_list))

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
            no_of_slice = (int(length / (L_max )) + 1) * k
            placement_dist = length / no_of_slice
            for it in range(1 ,  no_of_slice):


            # placement_dist = length / (int(length / L_max) + 1)
            # for it in range(1 ,  int(length / L_max) + 1):

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
                if i == 'LBNL' or j == 'LBNL':
                    print(i , j , placement_dist , length1 , dist , center1 , center2 , get_distance(center1 , i) , G[i][j]['length'])

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


def add_quantum_repeater( G , center_nodes , mandatory_centers , L_max , k):
    print("=============add_quantum_repeater========= number of nodes 1 " , G.number_of_nodes() , " ===================================")
    # print("type: " , G.nodes["SNLCA"]['type'])
    edge_list = compute_edges_to_place_new_repeaters(G , G.nodes() , k)

    print("edge list len " , len(edge_list))

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
            no_of_slice = (int(length / (L_max )) + 1) * k
            placement_dist = length / no_of_slice
            print('placement dist:: ' , placement_dist , length)
            for it in range(1 ,  no_of_slice):


            # placement_dist = length / (int(length / L_max) + 1)
            # for it in range(1 ,  int(length / L_max) + 1):

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
                if i == 'LBNL' or j == 'LBNL':
                    print(i , j , placement_dist , length1 , dist , center1 , center2 , get_distance(center1 , i) , G[i][j]['length'])

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
        print(i , '-' , j)
        try:
            G.remove_edge(i , j)
        except:
            print()
    G.add_edges_from(q_node_edges)
    for i, j in G.edges():
        if 'length' not in G[i][j]:
                _compute_dist_lat_lon(G)
    nx.set_node_attributes(G, pos, name='pos')
    print("==========add_quantum_repeater============ number of nodes 2 " , G.number_of_nodes() , " ===================================")
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
def calculate_children( G , L_max):
    print('in calculate children------------------' , len(shortest_path_dict))
    children_dict = {}
    for node in G.nodes():
        children_dict[node] = set()
    for i , j in shortest_path_dict:
        dist = get_distance(i , j)
        if dist <= L_max:
            if i in children_dict and j in G.nodes():
                children_dict[i].add(j)
            if j in children_dict and i in G.nodes():
                children_dict[j].add(i)
    
    # print(children_dict)
    return children_dict
def calculate_tree(G , center_nodes , L_max , source):
    children_dict = calculate_children( G , L_max)
    # children = children_dict[source]
    # print("===+++=== len of children " , len(children))
    root = nodeTree(source , None)
    # root.children = list()
    tmp_queue = []
    tmp_queue.append(root)
    visited = []
    visited.append(source)



    # for child in children:
    #     root.children.append(nodeTree(child))

    while tmp_queue:
        current_node = tmp_queue.pop(0)
        print("in while loop for " , current_node.node)
        # visited.append(c)

        # current_node = get_current_node(root , c)
        if current_node.children is None:
            current_node.children = list()
        children = children_dict[current_node.node]
        path = extract_path(current_node)
        print("len of path " , len(path))
        print("len of tmp queue " , len(tmp_queue))
        
        for child in children:
            if child not in path:
                new_node = nodeTree(child , current_node)
                current_node.children.append(new_node)
                if new_node not in tmp_queue:
                    tmp_queue.append(new_node)
        # if c == 'Venlo':
        #     print('len children for ' , c , len(children))
        #     # print(tmp_queue)
        #     print_node(current_node)
    return root
    
def get_all_path(G , center_nodes , L_max , source , target):
    root = calculate_tree(G , center_nodes , L_max , source)
  
    
    tmp_queue = []
    tmp_queue.append(root)
    paths = []
    

    while tmp_queue:
        c = tmp_queue.pop(0)
        if c.node == target:
            # print_node(c)
            paths.append(extract_path(c))
        else:
            if c.children is not None:
                for child in c.children:
                    tmp_queue.append(child)
    # for path in paths:
    #     print(path)

    paths2 = list(nx.all_simple_paths(G, source=source, target=target))
    print("len from simple path " , len(paths2))
    print("len from new path " , len(paths))

    return paths
def print_node(node):
    print("-------------node------------")
    # print(node.parent.node)
    print(node.node)
    if node.children is not None:
        for child in node.children:
            print(child.node)
    print("-------------------------")
    

def extract_path(nodeTree):
    path = list()
    path.append(nodeTree.node)
    parent_node = nodeTree.parent
    while parent_node is not None:
        path.append(parent_node.node)
        parent_node = parent_node.parent

    # print("==== " , path[::-1])
    return path[::-1]

def get_current_node(root , node):
    current_node = root
    tmp_queue = []
    tmp_queue.append(root)
    while tmp_queue:
        c = tmp_queue.pop(0)
        if c.node == node:
            return c
        else:
            if c.children is not None:
                for child in c.children:
                    tmp_queue.append(child)

    return current_node

def common_member(a, b):   
    a_set = set(a)
    b_set = set(b)
     
    # check length
    if len(a_set.intersection(b_set)) > 0:
        return(a_set.intersection(b_set)) 
    else:
        return None
def find_end_nodes(G):
    end_nodes = []
    for node in G.nodes():
        if G.degree(node) == 1:
            end_nodes.append(node)
            G.nodes[node]['type'] = 'end_node'
    # for node in end_nodes:
    #     G.remove_node(node)




    
