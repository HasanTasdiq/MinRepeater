import psutil
import os
import networkx as nx
from graph_tools import shortest_path_dict 

no_of_thread = 60

def kill_processes(pid):
    parent = psutil.Process(pid)
    for child in parent.children(recursive=True):
        print(child)
        child.kill()
    parent.kill()

def kill_all():
    pid = os.getppid()
    kill_processes(pid)

def calculate_output(G):
    global no_of_thread
    out_set = set()
    new_repeater_count = 0
    for thread_no in range(0 , no_of_thread):
        fo = open("center/chosen_" + str(thread_no) +".txt", "r")
        lines = fo.readlines()
        for line in lines:
            elements = line.split(" ") 
            out_set.update(elements)
        fo.close()
    
    for node in out_set:
        if node !=' ' and G.nodes[node]["type"] == "new_repeater_node":
            new_repeater_count += 1

    print("total repeater chosen:" , len(out_set))
    print('-------------------')
    print(out_set)
    print('---------=====------------')
    print("new repeaters needed:" , new_repeater_count)

    return out_set

def get_nearest_node(G , node):
    length = nx.single_source_shortest_path_length(G ,source=node, cutoff=1)

    for n in length:
        if n!=node:
            return n
    

