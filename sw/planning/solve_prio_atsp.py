from tsp_solver_lns_cleanup import TSPSolver, _create_test_graph, CostParams
import numpy as np
import argparse

def read_priorities(file):
    try:
        with open(file) as f:
            priorities = np.array(f.read().split(' '))
    except FileNotFoundError:
        raise
    return priorities


def write_tour(tour, file):
    dim = len(tour)
    
    try:
        with open(file, 'w') as f:
            tour_str = "" 
            for i in tour: tour_str += str(i+1) + "\n" 
            fstr = f"NAME : single.599.tour\nCOMMENT : Length = 599\nCOMMENT : Found by LKH [Keld Helsgaun] Fri May  3 11:55:16 2024\nTYPE : TOUR\nDIMENSION : {dim}\nTOUR_SECTION\n{tour_str}-1\nEOF\n"
            f.write(fstr)

    except FileNotFoundError:
        raise


def solve_prio_atsp(priorities):

    prio_factor = 1
    # priorities = priorities/priorities.sum()
    graph_size = 3
    G = _create_test_graph(
        graph_size,
        prio_factor,
        seed=4,
    )
    starts = [0]

    heuristic = "2opt_lns"
    param = dict(
        dist_unvisited=0, 
        heuristic=heuristic,
        lns_rounds=100,
        lns_max_del=0.3,
        lns_greedy_init=False,
        append_unvisited=True,
        randomize_insertion_cost=False,
        random_cost_factor=0.01,
        dist_mat_path = "/root/thesis_ws/src/thesis/sw/bringup/resource/single.tsp",
        node_priorities = priorities
    )

    # Create tsp object
    tsp = TSPSolver(
        G,
        **param,
    )
    if not len(priorities) == len(tsp.dist):
        raise RuntimeError("Number of priorities should be same as matrix size")
    
    print("Compiling...")
    tours, cost = tsp.solve_mtsp(starts)
    # print(tours, cost)
    return tours[0], cost

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("priorities", metavar='N', type=float, nargs="+")

    args = parser.parse_args()
    try:
        # priorities = read_priorities("/root/thesis_ws/src/thesis/sw/bringup/resource/priorities.txt")
        # print(args.priorities==None)
        tour, cost = solve_prio_atsp(args.priorities)
        write_tour(tour, "/root/thesis_ws/src/thesis/sw/bringup/resource/single.txt")
    except:
        raise