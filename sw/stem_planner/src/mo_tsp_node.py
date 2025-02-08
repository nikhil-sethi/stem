import rospy
from stem_planner.tsp_solver_lns_cleanup import TSPSolver
from stem_msgs.srv import TSP, TSPResponse
import numpy as np
import time

class MultiObjectiveTSP():
    def __init__(self) -> None:
        rospy.init_node("mo_tsp_node")
        np.random.seed(0)
        self.solve_server = rospy.Service("/planning/motsp_service", TSP, self.handle_solver)

        # warm start to load up in cache
        self.solve_prio_atsp(np.zeros((10,10)), np.ones(10), 10)
        rospy.spin()

    def handle_solver(self, req):
        priorities = list(req.priorities) # deserialised as tuple
        priorities.insert(0,0) # add any priority for self node
        priorities = 10*np.round(np.array(priorities)/10)
        cost_mat = self.mat_from_list(req.cost_mat_flat, req.dim)
        tour, cost = self.solve_prio_atsp(cost_mat, priorities, req.dim)
        return TSPResponse(tour, cost)
    
    @staticmethod
    def mat_from_list(mat_flat, dim):
        cost_mat = np.zeros((dim,dim))
        iu = np.triu_indices(dim,1)
        cost_mat[iu] = mat_flat
        cost_mat[iu[::-1]] = mat_flat
        return cost_mat

    @staticmethod
    def solve_prio_atsp(cost_mat, priorities, dim):
        # start = time.perf_counter()
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
            dist_mat = cost_mat,
            node_priorities = priorities,
            nodes = range(dim)
        )

        # Create tsp object
        tsp = TSPSolver(
            None,
            **param,
        )
        if not len(priorities) == len(tsp.dist):
            raise RuntimeError("Number of priorities should be same as matrix size")
        
        # print("Compiling...")
        
        tours, cost = tsp.solve_mtsp(starts)
        tour = [tours[0][i]-1 for i in range(1,len(tours[0]))] # remove first position and readjust. FUEL expects this
        # print("tour: ", tour)
        # print("Time: ", time.perf_counter()-start)
        return tour, cost


if __name__=="__main__":
    motsp_node = MultiObjectiveTSP()