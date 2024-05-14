import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import timeit
from tqdm import tqdm
from numba import jit
from numba import config as nbconfig
from numba.experimental import jitclass

from numba.core import types
from typing import List, Tuple, Dict, Optional, Union

import warnings
from numba.core.errors import NumbaDeprecationWarning, NumbaPendingDeprecationWarning

warnings.simplefilter("ignore", category=NumbaDeprecationWarning)
warnings.simplefilter("ignore", category=NumbaPendingDeprecationWarning)

# nbconfig.DISABLE_JIT = True

cost_param_spec = [
    ("dist_matrix", types.float64[:, :]),
    ("priorities", types.float64[:]),
    ("use_priority", types.boolean),
    ("use_waiting_time", types.boolean),
    ("use_edge_priority", types.boolean),
    ("dist_unvisited", types.float64),
    ("append_unvisited", types.boolean),
    ("edge_prio_factor", types.float64),
]


@jitclass(cost_param_spec)
class CostParams:
    def __init__(
        self,
        dist_matrix: np.ndarray,
        priorities: np.ndarray,
        use_priority: bool,
        use_waiting_time: bool,
        dist_unvisited: float,
        append_unvisited: bool,
    ):
        self.dist_matrix = dist_matrix
        self.priorities = priorities
        self.use_priority = use_priority
        self.use_waiting_time = use_waiting_time
        self.dist_unvisited = dist_unvisited
        self.append_unvisited = append_unvisited


@jit(nopython=True, cache=True)
def _tour_cost(
    tour: np.ndarray,
    visited_nodes_idxs: np.ndarray,
    nodes: np.ndarray,
    cost_params: CostParams,
):
    r"""
    Compute the cost of a tour
    Compute waiting time for each node in the tour, weighted by the priority of the node
    .. math::
        \sum_{i=1}^{n-1} (I_i * \sum_{j=1}^{i} d_{ij})
    """

    dist_matrix = cost_params.dist_matrix
    priorities = cost_params.priorities
    use_priority = cost_params.use_priority
    use_waiting_time = cost_params.use_waiting_time
    dist_unvisited = cost_params.dist_unvisited
    append_unvisited = cost_params.append_unvisited

    # Get list of unvisited nodes
    unvisited_nodes = np.array(
        [i for i in range(len(nodes)) if i not in visited_nodes_idxs and i not in tour]
    )

    # Add unvisited nodes to tour
    if append_unvisited and len(unvisited_nodes) > 0:
        new_tour = np.zeros(len(tour) + len(unvisited_nodes), dtype=np.int64)
        new_tour[: len(tour)] = tour
        new_tour[len(tour) :] = unvisited_nodes

    cum_dist = np.zeros(len(tour))
    for i in range(1, len(tour)):
        cum_dist[i] = cum_dist[i - 1] + dist_matrix[tour[i - 1], tour[i]]

    if use_priority and use_waiting_time:
        sum_list = np.array(
            [priorities[tour[i]] * cum_dist[i] for i in range(len(tour))]
        )
        cost = np.sum(sum_list)

        # Add cost for unvisited nodes
        if dist_unvisited > 0:
            sum_list2 = np.array(
                [priorities[n] * dist_unvisited for n in unvisited_nodes]
            )
            cost += np.sum(sum_list2)

    elif use_waiting_time:
        cost = np.sum(cum_dist)
    else:
        cost = cum_dist[-1]

    cost /= len(tour)

    return cost


@jit(nopython=True, cache=True)
def _tsp_insertion(
    nodes: np.ndarray,
    tour: np.ndarray,
    visited_nodes_idxs: np.ndarray,
    tour_cost: float,
    cost_params: CostParams,
    random_cost: bool = False,
    random_cost_factor: float = 0.1,
    rng: np.random.Generator = np.random.default_rng(),
):
    # Iterate until all nodes are inserted
    added_node_idx = -1
    tour_length = len(tour)
    while tour_length < len(nodes):
        # Select next node to add to tour
        min_cost_overall = np.inf
        best_tour_overall = np.array([-1], dtype=np.int64)

        # iterate over all nodes that could be added
        for i in range(len(nodes)):
            # Skip nodes already in tour
            if i in visited_nodes_idxs:
                continue

            # Find cheapest insertion for node i
            min_cost_insertion = np.inf
            min_tour_insertion = np.array([-1], dtype=np.int64)
            for j in range(len(tour)):
                # Insert node i between nodes j and j+1
                new_tour = np.concatenate((tour[: j + 1], np.array([i]), tour[j + 1 :]))
                new_cost = _tour_cost(new_tour, visited_nodes_idxs, nodes, cost_params)
                # Add random cost
                if random_cost:
                    new_cost += rng.normal(0, random_cost_factor)
                # Update best tour for node i
                if new_cost < min_cost_insertion:
                    min_cost_insertion = new_cost
                    min_tour_insertion = new_tour

            # Update best insertion over all nodes given best insertion for node i
            if min_cost_insertion < min_cost_overall:
                min_cost_overall = min_cost_insertion
                best_tour_overall = min_tour_insertion
                added_node_idx = i

        # Update tour with best insertion over all nodes
        tour = best_tour_overall
        tour_cost = min_cost_overall
        visited_nodes_idxs = np.unique(tour)
        tour_length = len(tour)

        if tour is None:
            raise ValueError("Tour is None")

        # Compute cost of tour
        tour_cost = _tour_cost(tour, visited_nodes_idxs, nodes, cost_params)

    return tour, tour_cost, visited_nodes_idxs


@jit(nopython=True, cache=True)
def _tsp_large_neighborhood(
    start: int,
    nodes: np.ndarray,
    cost_params: CostParams,
    greedy_init: bool = True,
    deletion_factor: float = 0.5,
    deletion_rounds: int = 1000,
    rng: np.random.Generator = np.random.default_rng(),
    random_cost: bool = False,
    random_cost_factor: float = 0.1,
    two_opt: bool = False,
):
    """
    Solve the TSP problem using the large neighborhood search heuristic
    (only single agent)
    """

    # Initialize tour with a start node
    tour = np.array([start])
    visited_nodes_idxs = np.array([start])

    # Compute cost of tour
    tour_cost = _tour_cost(tour, visited_nodes_idxs, nodes, cost_params)

    if greedy_init:
        # Construct initial tour with greedy ordering
        priorities = cost_params.priorities
        prio_order = np.argsort(priorities)[::-1]
        # Remove start node from prio_order
        prio_order = [n for n in prio_order if n != start]
        # Add start nodes to beginning of priority order
        tour = np.concatenate((tour, np.array(prio_order)))
        visited_nodes_idxs = np.unique(tour)
        tour_cost = _tour_cost(tour, visited_nodes_idxs, nodes, cost_params)

    else:
        # Construct initial tours with cheapest insertion heuristic
        tour, tour_cost, visited_nodes_idxs = _tsp_insertion(
            nodes=nodes,
            tour=tour,
            visited_nodes_idxs=visited_nodes_idxs,
            tour_cost=tour_cost,
            cost_params=cost_params,
            random_cost=False,
            rng=rng,
        )

    # Perform 2-opt local search
    if two_opt:
        tour, tour_cost = _tsp_2opt(
            tour=tour,
            nodes=nodes,
            cost_params=cost_params,
        )

    # Large neighborhood search
    # Keep track of best tours
    best_tour = tour.copy()
    best_tour_cost = tour_cost

    # set maximum number of deletions
    n_deletions_max = int(deletion_factor * len(nodes))

    # If no deletions, skip deletion rounds and return initial tours
    if n_deletions_max == 0:
        return best_tour, best_tour_cost

    # randomly delete and insert for fixed number of rounds
    n_improvements = 0
    last_improvement = 0
    for round in range(deletion_rounds):

        # Randomly delete N nodes from tours and reinsert them
        n_deletions = rng.integers(1, n_deletions_max + 1)
        for _ in range(n_deletions):
            node_idx = rng.integers(1, len(tour))

            # Remove node_idx from tour
            tour = np.delete(tour, node_idx)
            # Remove node from visited nodes
            visited_nodes_idxs = np.unique(tour)

        # Reinsert deleted nodes
        tour, tour_cost, visited_nodes_idxs = _tsp_insertion(
            nodes=nodes,
            tour=tour,
            visited_nodes_idxs=visited_nodes_idxs,
            tour_cost=tour_cost,
            cost_params=cost_params,
            random_cost=random_cost,
            random_cost_factor=random_cost_factor,
            rng=rng,
        )

        # Perform 2-opt local search
        if two_opt:
            tour, tour_cost = _tsp_2opt(
                tour=tour,
                nodes=nodes,
                cost_params=cost_params,
            )

        # Update best tours if new best
        if tour_cost < best_tour_cost:
            best_tour = tour.copy()
            best_tour_cost = tour_cost
            n_improvements += 1
            last_improvement = round

        # Otherwise, revert to best tours
        else:
            tour = best_tour.copy()
            tour_cost = best_tour_cost

    # print("Number of improvements: ", n_improvements)
    # print("Last improvement round", last_improvement)

    return best_tour, best_tour_cost


@jit(nopython=True, cache=True)
def _tsp_2opt(
    tour: np.ndarray,
    nodes: np.ndarray,
    cost_params: CostParams,
):
    """
    Perform 2-opt local search (single agent)
    """

    # Compute initial cost
    best_cost = _tour_cost(tour, set(tour), nodes, cost_params)
    best_tour = tour.copy()

    is_improved = True
    while is_improved:
        is_improved = False
        for i in range(1, len(tour) - 1):
            for j in range(i + 1, len(tour)):
                new_tour = best_tour.copy()
                new_tour[i : j + 1] = new_tour[i : j + 1][::-1]
                new_cost = _tour_cost(new_tour, set(new_tour), nodes, cost_params)
                if new_cost < best_cost:
                    best_tour = new_tour.copy()
                    best_cost = new_cost
                    is_improved = True

    return best_tour, best_cost


@jit(nopython=True, cache=True)
def _solve_wrapper(
    heuristic: str,
    start_idx: np.ndarray,
    nodes: np.ndarray,
    lns_greedy_init: bool,
    lns_rounds: int,
    lns_max_del: float,
    rng: np.random.Generator,
    dist_matrix: np.ndarray,
    priorities: np.ndarray,
    use_priority: bool,
    use_waiting_time: bool,
    dist_unvisited: float,
    append_unvisited: bool,
    random_cost: bool = False,
    random_cost_factor: float = 0.1,
):
    """
    Wrapper function for jitted heuristic functions
    - improves compilation time by keeping the CostParams jitclass inside jitted function
    """
    cost_params = CostParams(
        dist_matrix,
        priorities,
        use_priority,
        use_waiting_time,
        dist_unvisited,
        append_unvisited,
    )

    if heuristic == "insertion":
        tours, cost = _tsp_large_neighborhood(
            start=start_idx,
            nodes=nodes,
            cost_params=cost_params,
            greedy_init=False,
            deletion_rounds=0,
            rng=rng,
            random_cost=False,
            two_opt=False,
        )
    elif heuristic == "lns":
        tours, cost = _tsp_large_neighborhood(
            start=start_idx,
            nodes=nodes,
            cost_params=cost_params,
            greedy_init=lns_greedy_init,
            deletion_factor=lns_max_del,
            deletion_rounds=lns_rounds,
            rng=rng,
            random_cost=random_cost,
            random_cost_factor=random_cost_factor,
            two_opt=False,
        )
    elif heuristic == "2opt":
        tours, cost = _tsp_large_neighborhood(
            start=start_idx,
            nodes=nodes,
            cost_params=cost_params,
            greedy_init=False,
            deletion_rounds=0,
            rng=rng,
            random_cost=False,
            two_opt=True,
        )
    elif heuristic == "2opt_lns":
        tours, cost = _tsp_large_neighborhood(
            start=start_idx,
            nodes=nodes,
            cost_params=cost_params,
            greedy_init=lns_greedy_init,
            deletion_factor=lns_max_del,
            deletion_rounds=lns_rounds,
            rng=rng,
            random_cost=random_cost,
            random_cost_factor=random_cost_factor,
            two_opt=True,
        )

    return tours, cost


class TSPSolver:
    """
    TSP solver using the cheapest insertion heuristic

    Parameters
    ----------
    graph : nx.Graph
        Graph to solve TSP on
    nodes : list, optional
        List of nodes to solve TSP on, by default None (then all nodes in graph)
    use_priority : bool, optional
        Use priority attribute of nodes, by default True
    use_waiting_time : bool, optional
        Use waiting time cost function, by default True, otherwise use total distance
    dist_unvisited : int, optional
        Distance cost for unvisited nodes, by default 1e6
    edge_weight : str, optional
        Edge weight attribute key, by default "weight"
    """

    def __init__(
        self,
        graph: nx.Graph,
        nodes: Optional[List[int]] = None,
        use_waiting_time: bool = True,
        use_priority: bool = True,
        priority_key: Optional[str] = "priority",
        node_priorities: Optional[np.ndarray] = None,
        dist_unvisited: float = 0,
        append_unvisited: bool = False,
        edge_weight_key: str = "weight",
        print_debug: bool = False,
        heuristic: str = "2opt_lns",
        lns_greedy_init: bool = False,
        lns_rounds: int = 1000,
        lns_max_del: float = 0.5,
        rng: np.random.Generator = np.random.default_rng(),
        randomize_insertion_cost: bool = False,
        random_cost_factor: float = 0.1,
        dist_mat_path = None
    ):
        self.graph = graph
        self.use_priority = use_priority
        self.priority_key = priority_key
        self.use_waiting_time = use_waiting_time
        self.dist_unvisited = dist_unvisited
        self.append_unvisited = append_unvisited
        self.edge_weight_key = edge_weight_key
        self.print_debug = print_debug
        self.heuristic = heuristic
        self.lns_greedy_init = lns_greedy_init
        self.lns_rounds = lns_rounds
        self.lns_max_del = lns_max_del
        self.rng = rng
        self.randomize_insertion_cost = randomize_insertion_cost
        self.random_cost_factor = random_cost_factor

        if nodes is None:
            self.nodes = np.array(list(self.graph.nodes))
            self.removed_nodes = []
        else:
            self.nodes = np.array(nodes)
            self.removed_nodes = [n for n in self.graph.nodes if n not in self.nodes]

        if dist_mat_path is None:
            # Compute distance matrix
            self.dist = self.compute_distance_matrix()
        else:
            self.dist = self.load_distance_matrix(dist_mat_path)
            self.nodes = np.array(list(range(self.dist.shape[0])))
            print(self.nodes)

        # Store priorities
        if self.use_priority:
            if node_priorities is not None:
                self.priorities = np.array(node_priorities)
            else:
                self.priorities = np.array(
                    [self.graph.nodes[i][self.priority_key] for i in self.nodes]
                )

            self.priorities = self.priorities.astype(np.float64)
        else:
            self.priorities = np.array([], dtype=np.float64)

    def solve_mtsp(self, starts: List[int] = None) -> Tuple[List[List[int]], float]:
        """
        Solve the TSP problem with a cheapest insertion heuristic
        :param start
        :return: tour, cost
        """

        # Modify distance matrix to make sure that the start node is always the first node in the tour
        start_indices = []

        if self.print_debug:
            print(
                "Nodes in mtsp: length = {0}, nodes = {1}".format(
                    len(self.nodes), self.nodes
                )
            )
            for i, start in enumerate(starts):
                print("Start {0}: {1}".format(i, start))

        for start in starts:
            start_idx = np.where(self.nodes == start)[0][0]
            self.dist[:, start_idx] = 0
            start_indices.append(start_idx)

        # Solve TSP problem
        tour, cost = _solve_wrapper(
            self.heuristic,
            np.array(start_indices)[0],
            self.nodes,
            self.lns_greedy_init,
            self.lns_rounds,
            self.lns_max_del,
            self.rng,
            self.dist,
            self.priorities,
            self.use_priority,
            self.use_waiting_time,
            self.dist_unvisited,
            self.append_unvisited,
            self.randomize_insertion_cost,
            self.random_cost_factor,
        )

        # Convert tour from idx to node id
        tours = [tour]
        tours_ids = []
        for tour in tours:
            tours_ids.append(self.nodes[tour].tolist())

        if self.print_debug and len(tours_ids) > 1:
            print(
                "Tour lengths: length 0 = {0}, length 1 = {1}, length nodes = {2}".format(
                    len(tours_ids[0]), len(tours_ids[1]), len(self.nodes)
                )
            )

        return tours_ids, cost

    def compute_distance_matrix(self) -> np.ndarray:
        """
        Compute the distance matrix for all nodes in the graph
        - initialize and run floyd-warshall algorithm
        """

        # Use NX Floyd-Warshall algorithm
        dist_all = nx.floyd_warshall_numpy(self.graph, weight=self.edge_weight_key)

        # Get distance matrix for subgraph defined by self.nodes
        node_idc = np.array([np.where(self.graph.nodes == n)[0][0] for n in self.nodes])
        dist = dist_all[node_idc, :][:, node_idc]

        return dist

    def load_distance_matrix(self, path) -> np.ndarray:
        """Read LKH format cost matrix"""
        with open(path) as f:
            mat = []
            lines = f.readlines()
            
            dim = int(lines[2][-2])
            for i in range(dim):
                row = np.array((lines[6+i][:-1]).split(' ')[:-1], dtype=int)
                mat.append(row)
        arr = np.array(mat, dtype=float)
        return arr # norrmalise it to fit with this code


def _create_test_graph(N, prio_factor=1, seed=0):
    # Create a graph with N nodes, and random positions in the unit square
    # G = nx.complete_graph(N)
    G = nx.random_geometric_graph(N, radius=15 / N, seed=seed)

    # Check if graph is connected
    if not nx.is_connected(G):
        return None

    np.random.seed(seed)

    # high_priority_nodes = [1]

    # Generate or convert positions
    for node in G.nodes:
        # G.nodes[node]["pos"] = np.random.rand(2)
        G.nodes[node]["pos"] = np.array(G.nodes[node]["pos"])

    # Add priority to nodes
    for node in G.nodes:
        # G.nodes[node]["priority"] = 1 if node in high_priority_nodes else 0.01
        G.nodes[node]["priority"] = np.random.rand() * prio_factor
        # G.nodes[node]["priority"] = 1

        # Add distance between nodes as weight to all edges
    for u, v in G.edges:
        G[u][v]["weight"] = np.linalg.norm(G.nodes[u]["pos"] - G.nodes[v]["pos"])

    return G


def test_mtsp():

    # Create graph
    prio_factor = 1
    graph_size = 5
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
    )

    # Create tsp object
    tsp = TSPSolver(
        G,
        **param,
    )

    # Solve TSP problem
    print("Compiling...")
    tours, cost = tsp.solve_mtsp(starts)

    costs = []

    for i in tqdm(range(1)):

        rng = np.random.default_rng(seed=i)

        # Create graph
        G = _create_test_graph(graph_size, prio_factor, seed=i)
        G.nodes[3]["priority"] = 100
        if G is None:
            continue

        # Create tsp object
        tsp = TSPSolver(G, rng=rng, **param)
        # time_start = timeit.default_timer()
        tours, cost = tsp.solve_mtsp(starts)
        # time_elapsed = timeit.default_timer() - time_start
        # print("Heuristic time elapsed: ", time_elapsed)
        # for tour in tours:
        # print("Tour: ", tour)

        # print("Tour cost: ", cost)

        costs.append(cost)

    print("Avg cost: ", str(np.mean(costs)))

    # # Plot the graph with node indices
    fig, ax = plt.subplots()
    # Color nodes according to priority
    node_colors = [G.nodes[n]["priority"] for n in G.nodes]
    pos = nx.get_node_attributes(G, "pos")
    nx.draw_networkx_nodes(G, pos, node_color=node_colors, cmap=plt.cm.jet, ax=ax)
    idc = np.arange(len(G.nodes))
    labels = dict(zip(G.nodes, idc))
    nx.draw_networkx_labels(G, pos, labels, font_size=16, font_color="gray")

    # # Plot the edges
    nx.draw_networkx_edges(G, pos, alpha=0.5, ax=ax)

    # # Plot the tour
    colors = ["r", "g", "b"]
    for idx, tour in enumerate(tours):
        tour_edges = [
            (tour[i], tour[(i + 1) % len(tour)]) for i in range(len(tour) - 1)
        ]
        nx.draw_networkx_edges(
            G, pos, edgelist=tour_edges, width=8, alpha=0.5, edge_color=colors[idx]
        )
        # Label edges by tour order
        tour_labels = dict(zip(tour_edges, range(len(tour))))
        nx.draw_networkx_edge_labels(
            G,
            pos,
            edge_labels=tour_labels,
            font_size=12,
            font_color=colors[idx],
            label_pos=0.5,
            verticalalignment="top",
        )

    # # Get min and max x and y values from all nodes
    # x_min = min([pos[node][0] for node in pos])
    # x_max = max([pos[node][0] for node in pos])
    # y_min = min([pos[node][1] for node in pos])
    # y_max = max([pos[node][1] for node in pos])
    # # Set plot limits
    # ax.set_xlim(x_min - 0.1, x_max + 0.1)
    # ax.set_ylim(y_min - 0.1, y_max + 0.1)

    # # Display colorbar
    priorities = [G.nodes[n]["priority"] for n in G.nodes]
    sm = plt.cm.ScalarMappable(
        cmap=plt.cm.jet, norm=plt.Normalize(vmin=min(priorities), vmax=max(priorities))
    )
    sm._A = []
    cbar = plt.colorbar(sm)

    plt.axis("on")  # turns on axis
    plt.grid(True)  # turns on grid
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    ax.set_aspect("equal")
    plt.show()

    # a = 1



if __name__ == "__main__":
    test_mtsp()
    # test_lkh_mtsp()
