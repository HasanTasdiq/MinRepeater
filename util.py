import psutil
import os
from algorithms import no_of_thread

def kill_processes(pid):
    parent = psutil.Process(pid)
    for child in parent.children(recursive=True):
        print(child)
        child.kill()
    parent.kill()

def kill_all():
    pid = os.getppid()
    kill_processes(pid)

def calculate_output():
    out_set = set()
    for thread_no in range(0 , no_of_thread):
        fo = open("center/chosen_" + thread_no +".txt", "r")
        lines = fo.readlines()
        for line in lines:
            elements = line.split(" ") 
            out_set.update(elements)
        fo.close()
    
    print("total repeater chosen:" , len(out_set))
