
from graph_tools import draw_graph, read_graph_from_gml , compute_shortest_path
from algorithms import compute_center_nodes

def solve(G):
    center_nodes = compute_center_nodes(G)

def main(file_name):
    G = read_graph_from_gml(file_name)
    compute_shortest_path
    draw_graph(G)


if __name__ == "__main__":
    main('es_net.gml')