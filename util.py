import psutil
import os
import networkx as nx
from graph_tools import shortest_path_dict 

no_of_thread = 6

def kill_processes(pid):
    parent = psutil.Process(pid)
    for child in parent.children(recursive=True):
        print(child)
        child.kill()
    parent.kill()

def kill_all():
    pid = os.getppid()
    kill_processes(pid)

def calculate_capacity(capacity_map , node_list):
    for node in node_list:
        if node =='':
            continue
        if node in capacity_map:
            capacity_map[node] = capacity_map[node] + 1
        else:
            capacity_map[node] = 1

def calculate_output(G):
    global no_of_thread
    out_set = set()
    capacity_map = {}
    new_repeater_count = 0
    for thread_no in range(0 , no_of_thread):
        fo = open("center/chosen_" + str(thread_no) +".txt", "r")
        lines = fo.readlines()
        for line in lines:
            elements = line.split(";")
            calculate_capacity(capacity_map , elements)
            out_set.update(elements)
        fo.close()
    out_set.remove('')
    for node in out_set:
        if (node !='' and node != ' ') and G.nodes[node]["type"] == "new_repeater_node":
            new_repeater_count += 1

   
    print('-------------------')
    print(capacity_map)
    print('---------=====------------')
    print("capacity map len " , len(capacity_map))
    print("total repeater chosen:" , len(out_set))
    print("new repeaters needed:" , new_repeater_count)
    print("existing repeaters needed:" , len(out_set) - new_repeater_count)

    return out_set

def get_nearest_node(G , node):
    length = nx.single_source_shortest_path_length(G ,source=node, cutoff=1)

    for n in length:
        if n!=node:
            return n
    

