import networkx as nx
from graph_tools import shortest_path_dict
import random

def compute_center_nodes(G , L_max , delta):
    center_nodes = set()
    nodes = list(G.nodes())
    print("len of nodes " , len(nodes))
    initial_center_node = nodes[random.randint(0, len(nodes) - 1)]
    center_nodes.add(initial_center_node)
    nodes.remove(initial_center_node)

    max_len = L_max * delta
    while max_len >= L_max * delta:
        dist = 0
        

        temp_list = set()

        for c_node in center_nodes:
            chosen_center = None
            max_len = L_max * delta
            for node in nodes:
                dist = get_distance(c_node , node)
                # print("dist " , dist , "m l" , max_len)
                if dist > max_len:
                    max_len = dist
                    chosen_center = node
            if chosen_center is not None:
                temp_list.add(chosen_center)
        
        if len(temp_list) > 0:
            print("::::" , temp_list)
            print("==== " , center_nodes)
            print("len of temp:" , len(temp_list))

        center , d = get_far_node_from_all_center(center_nodes , temp_list)

        if d > L_max * delta:
            center_nodes.add(center)
            # print("chosen " , center)
            nodes.remove(center)
            max_len = d
        else:
            break
        

    
    print(center_nodes)
    return center_nodes

def get_distance(node1 , node2 ):
    dist = -1
    if (node1 , node2) in shortest_path_dict:
        dist = shortest_path_dict[(node1 , node2)][0]
    if (node2 , node1) in shortest_path_dict:
        dist = shortest_path_dict[(node2 , node1)][0]
    return dist

def get_far_node_from_all_center(center_nodes , temp_list):
    
    
    dist = 0
    prev_dist = 0
    
    chosen_center2 = None
    for node  in temp_list:
        min_dist = 99999
        chosen_center = None
        for c_node in center_nodes:
            if get_distance(node , c_node) < min_dist:
                min_dist  = get_distance(node , c_node)
                chosen_center = node

        if min_dist > prev_dist and chosen_center is not None:
            chosen_center2 = chosen_center
            prev_dist = min_dist
    print("min dist " , prev_dist , chosen_center)
    return chosen_center2 , prev_dist



