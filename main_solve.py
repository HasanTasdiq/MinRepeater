
from graph_tools import draw_graph, read_graph_from_gml , compute_shortest_path , add_quantum_repeater
from algorithms import compute_center_nodes , compute_shortest_path_between_centers
from util import calculate_output
import time

def solve(G , L_max , delta):
    center_nodes = compute_center_nodes(G , L_max, delta)
    # critical_paths = compute_shortest_path_between_centers(G , center_nodes , L_max)

    draw_graph(G , center_nodes)

def main(file_name):
    L_max = 130
    delta =1/ 30
    G = read_graph_from_gml(file_name)
    print("read from graph done !!")
    add_quantum_repeater(G , L_max * 14 / 15  )
    print("add_quantum_repeater done!!")
    compute_shortest_path(G)

    print("compute_shortest_path done!!")
    solve(G , L_max , delta)
    print("calculating output")

    calculate_output(G)
    


if __name__ == "__main__":
    start = time.time()
    main('es_net.gml')
    # main('custom.gml')
    end = time.time()

    print("time taken:" , end - start , "sec")
