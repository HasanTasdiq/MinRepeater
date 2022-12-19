
from graph_tools import draw_graph, read_graph_from_gml , compute_shortest_path , add_quantum_repeater
from algorithms import compute_center_nodes , compute_shortest_path_between_centers

def solve(G , L_max , delta):
    center_nodes = compute_center_nodes(G , L_max, delta)
    # critical_paths = compute_shortest_path_between_centers(G , center_nodes , L_max)

    draw_graph(G , center_nodes)

def main(file_name):
    L_max = 136
    delta =1
    G = read_graph_from_gml(file_name)
    add_quantum_repeater(G , L_max - 16)
    compute_shortest_path(G)
    solve(G , L_max , delta)
    


if __name__ == "__main__":
    main('es_net.gml')
    # main('SurfnetCore.gml')