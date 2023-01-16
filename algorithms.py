import networkx as nx
from graph_tools import shortest_path_dict , get_distance_between_nodes, get_distance , draw_graph , compute_edges_to_choose_more_centers
from util import kill_all , no_of_thread , get_nearest_node
import random
import itertools
import threading
import math
from multiprocessing import Process
import sys
import os
from itertools import permutations 


terminate = False


def compute_center_nodes(G , L_max , delta , k):
    center_nodes = set()
    G2 = G.copy()

    for i in range(0 , k):
        center_nodes_i = get_center_nodes(G2 , L_max , delta )
        center_nodes.update(center_nodes_i)
        for c in center_nodes_i:
            G2.remove_node(c)
    return center_nodes

def get_center_nodes(G , L_max , delta ):
    # print("sssssss " , get_distance('A\'dam 2' , 'Westerbork'))
    # print("sssssss " , get_distance('PNNL' , 'NETL-ALB'))
    # print("sssssss " , get_distance('Nijmegen 1' , 'Utrecht 1'))
    center_nodes = set()
    nodes = list(G.nodes())
    # print("len of nodes " , len(nodes))
    # initial_center_node = None
    # for node in nodes:
    #     if G.degree[node] > 1:
    #         initial_center_node = node
    #         break

    # nodes.remove(initial_center_node)



    compute_mandatory_centers(G , center_nodes , L_max)

    if len(center_nodes) <= 1:
        # initial_center_node = nodes[random.randint(0, len(nodes) - 1)]
        # initial_center_node = "A'dam 2"
        initial_center_node = get_initial_node(G , L_max * delta)

        print('init center:', initial_center_node)
        center_nodes.add(initial_center_node)

    for c in center_nodes:
        if c in nodes:
            nodes.remove(c)
        remove_nodes_inside_circle(c , nodes , L_max*delta)
    skip_end_nodes(G , nodes)

    max_len = L_max * delta
    while max_len >= L_max * delta:
        # print("len of nodes " , len(nodes))

        dist = 0
        

        temp_list = set()

        for c_node in center_nodes:
            chosen_center = None
            max_len = L_max * delta
            for node in nodes:
                if node in temp_list:
                    # print("yes in temp")
                    continue
                if G.degree[node] <=1:
                    nearest_node = get_nearest_node(G , node)
                    d = get_distance(node , nearest_node)
                    # if len(center_nodes) == 1:
                    #     print(node , nearest_node , d)
                    if d <= L_max * delta :
                        continue
                dist = get_distance(c_node , node)
                # adding weight
                # if G.nodes[node]["type"] == "new_repeater_node":
                #     dist = dist * .7
                if dist > max_len:
                    max_len = dist
                    chosen_center = node
                    # print("dist " , dist , "m l" , max_len)

            if chosen_center is not None:
                temp_list.add(chosen_center)
        


        center , d = get_far_node_from_all_center(center_nodes , temp_list)

        if d >= L_max * delta:
            center_nodes.add(center)
            # print("chosen " , center , d)
            nodes.remove(center)
            max_len = d
            remove_nodes_inside_circle(center , nodes , L_max*delta)

        else:
            break
        

    
    print('center_nodes len' , len(center_nodes))

    # print("==== t d " , t_d)
    return center_nodes


def get_initial_node(G , radius):
    max_nodes_inside_circle = 0
    init_node = None
    for node1 in G.nodes():
        nodes_inside_circle = 0
        for node2 in G.nodes():
            if node1 != node2 and get_distance(node1 , node2) <= radius:
                nodes_inside_circle += 1
        if nodes_inside_circle > max_nodes_inside_circle:
            init_node = node1
            max_nodes_inside_circle = nodes_inside_circle

    return init_node
        

def get_end_nodes(G , node):
    end_nodes = []
    for i , j in  G.edges(node):
        if G.degree[j] == 1:
            end_nodes.append(j)

    return end_nodes
def node_mandatory_for_end_nodes(G , end_nodes , L_max):
    pairs = list(itertools.combinations(end_nodes, r=2))
    for pair in pairs:
        (path_cost, sp) = nx.single_source_dijkstra(G=G, source=pair[0], target=pair[1], weight='length')
        if path_cost > L_max:
            return True
    return False


def compute_mandatory_centers(G , center_nodes , L_max):
    for node in G.nodes():
        end_nodes = get_end_nodes(G , node)
        if len(end_nodes) <= 0:
            continue
        if node_mandatory_for_end_nodes(G , end_nodes , L_max):
            center_nodes.add(node)
    print('&&&&&&&&& mandatory ' , center_nodes)
    
def skip_end_nodes(G , nodes):
    for node in G.nodes():
        if G.degree(node) == 1:
            if node in nodes:
                nodes.remove(node)
            G.nodes[node]['type'] = 'end_node'

def get_far_node_from_all_center(center_nodes , temp_list):
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
    return chosen_center2 , prev_dist
def remove_nodes_inside_circle(center , nodes , radius):
    nodes_inside_circle = []
    for node in nodes:
        if get_distance(node , center) <= radius:
            nodes_inside_circle.append(node)
    
    for node in nodes_inside_circle:
        if node in nodes:
            nodes.remove(node)

def compute_shortest_path_between_centers(G , center_nodes , L_max):
    shortest_path_between_centers = []
    for i in center_nodes:
        for j in center_nodes:
            if i != j:
                # Use NetworkX to generate the shortest path with Dijkstra's algorithm
                (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
                if path_cost <= L_max / 2:
                    shortest_path_between_centers.append((i , j , sp))
    print('len of crit path set' , len(shortest_path_between_centers))
def is_feasible_path(path , center_nodes , L_max):
    
    first_node_of_link = path[0]
    current_node = None
    for i in range(1 , len(path) - 1):
        current_node = path[i]
        if current_node in center_nodes:
            dist = get_distance(first_node_of_link , current_node)
            if dist > L_max:
                # print(first_node_of_link , current_node , dist)
                # print(path)
                return False
            first_node_of_link = current_node
    last_node_of_link = path[-1]
    dist = get_distance(first_node_of_link , last_node_of_link)
    if dist > L_max:
        return False
    
    return True

# def check_pairs_2(G, center_nodes , L_max , unique_end_node_pairs , thread_no , k ):
#     if k ==1:
#         check_pairs_k(G2, center_nodes , L_max , unique_end_node_pairs , thread_no )
#     else:
#         random_down_nodes_ = list(itertools.combinations(center_nodes, r=k-1))
#         for tup in random_down_nodes_:
#             G2 = G.copy()
#             for i in range(k-1):
#                 G2.remove_node(tup[i])
#             check_pairs_k(G2, center_nodes , L_max , unique_end_node_pairs , thread_no )

            
            
def check_pairs(G, center_nodes , L_max , unique_end_node_pairs , thread_no ):

    solution_exists = True
    pair_no = 0
    for i , j in unique_end_node_pairs:
        print("running for pair:" , pair_no , i , j , "in thread:" , thread_no)

        paths = list(nx.all_simple_paths(G, source=i, target=j))
        paths.sort(key = len)

        feasible_path = None
        path_length = 9999
        for path in paths:
            # print(path)
            if is_feasible_path(path , center_nodes , L_max):
                # print("path len" , len(path))
                feasible_path = path
                put_in_file(path , center_nodes ,  thread_no)
                path_length = len(path)
                print('feasible path found for pair ' , pair_no)
                break
        if feasible_path is None:
            print("!!!!---------------- !!!!!!!!!!!! NOT feasible !!!!!!!!!! --------------- in thread: " , thread_no )
            # print(path)
            solution_exists = False
            kill_all()
        pair_no += 1
    if solution_exists:
        print("*********** solution exists!!!!!!!!!!!!!! in thread:" , thread_no)


def choose_as_center(G , center_nodes , L_max):

    center_pairs = list(permutations(center_nodes, 2))
    edge_list = compute_edges_to_choose_more_centers(G , center_nodes)
    print('edge list ' , edge_list)
    for i , j in edge_list:
        # i = pair[0]
        # j = pair[1]
        center1 = i
        (path_cost, sp) = nx.single_source_dijkstra(G=G, source=i, target=j, weight='length')
        if path_cost > L_max:
            node1 = i
            for n in range(1 , len(sp)):
                node2 = sp[n]
                (dist2, sp2) = nx.single_source_dijkstra(G=G, source=center1, target=node2, weight='length')
                if dist2 > L_max:
                    center_nodes.add(node1)
                    center1 = node1
                node1 = node2

    


def check_solution(G , center_nodes , L_max , k):
    end_nodes =  [x for x,y in G.nodes(data=True) if y['type']=="repeater_node" or y['type']=="end_node"]
    unique_end_node_pairs = list(itertools.combinations(end_nodes, r=2))
    print('len unique_end_node_pairs:' , len(unique_end_node_pairs))
    global no_of_thread
    slice_length = math.ceil(len(unique_end_node_pairs) / no_of_thread)
    print("len of slice:" , slice_length)
    thread_list = list()
    for i in range(no_of_thread):
        start_index = i * slice_length
        end_index = start_index + slice_length
        if end_index > len(unique_end_node_pairs):
            end_index = len(unique_end_node_pairs)
        sub_list = unique_end_node_pairs[start_index: end_index]
        t = Process(target=check_pairs, args=(G, center_nodes, L_max , sub_list , i ,k))
        print("running thread:::::: " , i)
        thread_list.append(t)
        t.start()
    for t in thread_list:
        t.join()
    
    new_node_count = 0
    for node in center_nodes:
        if G.nodes[node]["type"] == "new_repeater_node":
            new_node_count += 1
    print("new_node_count:", new_node_count )
    print("total quantum repeater needed:", len(center_nodes) )
    
def put_in_file(path , center_nodes , thread_no):
    f = open("center/chosen_" + str(thread_no) +".txt", "a")
    center_str = ""
    yes = False
    for node in path:
        if node in center_nodes:
            center_str += node + ";"
    

    f.write(center_str)
    f.close()


    
                




