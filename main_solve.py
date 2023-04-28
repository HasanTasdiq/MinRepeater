
from graph_tools import draw_graph, read_graph_from_gml , compute_shortest_path , add_quantum_repeater_between_centers , create_graph_on_unit_cube,add_quantum_repeater
from algorithms import compute_center_nodes  , check_solution , choose_as_center 
from util import calculate_output
import time

def solve(G , L_max , delta , k):
    add_quantum_repeater(G , set() , [] , L_max * .95  , k )
    compute_shortest_path(G)
    center_nodes , mandatory_centers = compute_center_nodes(G , L_max, delta , k)
    print("number of centers " , len(center_nodes) )
    add_quantum_repeater_between_centers(G , center_nodes , mandatory_centers , L_max * .95  , k )
    # print("add_quantum_repeater done!!")
    # print("number of centers after adding new rep " , len(center_nodes))

    # compute_shortest_path(G)


    print('---------------****888*******---------')
    print(center_nodes)
    # draw_graph(G , center_nodes)
    # center_nodes = set(['Zwolle 1', 'Nijmegen 1', 'Utrecht 1'])
    # draw_graph(G , center_nodes)
    choose_as_center(G , center_nodes , L_max, k)

    new_rep = 0
    for r in center_nodes:
        if 'QN' in r:
            new_rep = new_rep + 1
    

    print("number of final quantum repeaters " , len(center_nodes) , 'for' , L_max , k)
    print('new repeater:' , new_rep)





    # centers2 = set()
    # # centers2.update(["A'dam 2", 'Zwolle 1', 'Nijmegen 1', 'Utrecht 1', 'Utrecht 2'])
    # centers2.update(['Zwolle 1', 'Almere', 'Utrecht 1', 'Breda 1', 'Deventer'])

    # print(centers2)

    # check_solution(G , center_nodes , L_max , k)


    # center_nodes = ['Zwolle 1', 'Nijmegen 1', 'Utrecht 1']
    # draw_graph(G , center_nodes)

def main(file_name):
    L_max = 130
    delta =1
    k = 1
    G = read_graph_from_gml(file_name)
    # G = create_graph_on_unit_cube(n_repeaters=10, radius=0.6, draw=False, seed=9)

    # draw_graph(G , [])

    print("read from graph done !!")
    
    compute_shortest_path(G)

    print("compute_shortest_path done!!")
    solve(G , L_max , delta , k)
    # print("calculating output")

    # out_set = calculate_output(G)
    # draw_graph(G , out_set)


    


if __name__ == "__main__":
    start = time.time()
    main('es_net.gml')
    # main('SurfnetCore.gml')
    end = time.time()

    print("time taken:" , end - start , "sec")
