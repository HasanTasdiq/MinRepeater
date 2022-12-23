import networkx as nx
from graph_tools import shortest_path_dict
from util import kill_all
import random
import itertools
import threading
import math
from multiprocessing import Process
import sys
import os

terminate = False



def compute_center_nodes(G , L_max , delta):
    # print("sssssss " , get_distance('A\'dam 2' , 'Westerbork'))
    # print("sssssss " , get_distance('PNNL' , 'NETL-ALB'))
    # print("sssssss " , get_distance('Nijmegen 1' , 'Utrecht 1'))
    center_nodes = set()
    nodes = list(G.nodes())
    # print("len of nodes " , len(nodes))
    # initial_center_node = nodes[random.randint(0, len(nodes) - 1)]
    initial_center_node = nodes[0]
    center_nodes.add(initial_center_node)
    nodes.remove(initial_center_node)

    remove_nodes_inside_circle(initial_center_node , nodes , L_max*delta)

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
    check_solution(G , center_nodes , L_max)
    return center_nodes

def get_distance(node1 , node2 ):
    dist = -1
    if (node1 , node2) in shortest_path_dict:
        dist = shortest_path_dict[(node1 , node2)][0]
    if (node2 , node1) in shortest_path_dict:
        dist = shortest_path_dict[(node2 , node1)][0]
    return dist

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
        if get_distance(node , center) < radius:
            nodes_inside_circle.append(node)
    
    for node in nodes_inside_circle:
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
            if dist >= L_max:
                print(first_node_of_link , current_node , dist)
                print(path)
                return False
            first_node_of_link = current_node
    last_node_of_link = path[-1]
    dist = get_distance(first_node_of_link , last_node_of_link)
    if dist >= L_max:
        return False
    return True
            
            
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
                path_length = len(path)
                break
        if feasible_path is None:
            print("!!!!---------------- !!!!!!!!!!!! NOT feasible !!!!!!!!!! --------------- in thread: " , thread_no )
            solution_exists = False
            kill_all()
        pair_no += 1
    if solution_exists:
        print("*********** solution exists!!!!!!!!!!!!!! in thread:" , thread_no)
        new_node_count = 0
        for node in center_nodes:
            if G.nodes[node]["type"] == "new_repeater_node":
                new_node_count += 1
        print("new_node_count:", new_node_count )
        print("total quantum repeater needed:", len(center_nodes) )


def check_solution(G , center_nodes , L_max):
    end_nodes =  [x for x,y in G.nodes(data=True) if y['type']=="repeater_node"]
    unique_end_node_pairs = list(itertools.combinations(end_nodes, r=2))
    print('len unique_end_node_pairs:' , len(unique_end_node_pairs))
    no_of_thread = 1
    slice_length = math.ceil(len(unique_end_node_pairs) / no_of_thread)
    print("len of slice:" , slice_length)
    thread_list = list()
    for i in range(no_of_thread):
        start_index = i * slice_length
        end_index = start_index + slice_length
        if end_index > len(unique_end_node_pairs):
            end_index = len(unique_end_node_pairs)
        sub_list = unique_end_node_pairs[start_index: end_index]
        t = Process(target=check_pairs, args=(G, center_nodes, L_max , sub_list , i ))
        print("running thread:::::: " , i)
        thread_list.append(t)
        t.start()
    for t in thread_list:
        t.join()
    



    
                




