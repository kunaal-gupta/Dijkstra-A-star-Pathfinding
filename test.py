import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq


def test():
    """
    Function for testing your A* and Dijkstra's implementation.
    Run it with a -help option to see the options available.
    """
    optlist, _ = getopt.getopt(sys.argv[1:], 'h:m:r:', ['testinstances', 'plots', 'help'])

    plots = False
    for o, a in optlist:
        if o in ("-help"):
            print("Examples of Usage:")
            print("Solve set of test instances and generate plots: main.py --plots")
            exit()
        elif o in ("--plots"):
            plots = True

    test_instances = "test-instances/testinstances.txt"

    # Dijkstra's algorithm and A* should receive the following map object as input
    gridded_map = Map("dao-map/brc000d.map")

    nodes_expanded_dijkstra = []
    nodes_expanded_astar = []

    time_dijkstra = []
    time_astar = []

    start_states = []
    goal_states = []
    solution_costs = []

    file = open(test_instances, "r")
    for instance_string in file:
        list_instance = instance_string.split(",")
        start_states.append(State(int(list_instance[0]), int(list_instance[1])))
        goal_states.append(State(int(list_instance[2]), int(list_instance[3])))

        solution_costs.append(float(list_instance[4]))
    file.close()

    def Dijkstra(start, goal):
        OpenArr = [[0, start]]
        ClosedArr = {State(start.get_x(), start.get_y()).state_hash(): start}

        while len(OpenArr) != 0:
            n_cost, n = heapq.heappop(OpenArr)[:]
            parent_instance = State(n.get_x(), n.get_y())
            # print(n_cost, type(n_cost))
            parent_instance.set_g(n_cost)

            if n.get_x() == goal.get_x() and n.get_y() == goal.get_y():
                # print(ClosedArr)
                return heapq.heappop(OpenArr)[0], len(ClosedArr)

            for n_child in gridded_map.successors(n):
                n_x = n_child.get_x()
                n_y = n_child.get_y()
                child_instance = State(n_x, n_y)

                gOfN = parent_instance.get_g()
                costN_Nchild = gridded_map.cost(x=n_x, y=n_y)

                if child_instance.state_hash() not in ClosedArr:

                    heapq.heappush(OpenArr, [gOfN + costN_Nchild, n_child])                   # OpenList
                    ClosedArr[child_instance.state_hash()] = n_child                                  # ClosedList

                    child_instance.set_g(gOfN + costN_Nchild)

                if child_instance.state_hash() in ClosedArr and gOfN + costN_Nchild < child_instance.get_g():

                    print('hi')

                heapq.heapify(OpenArr)
        return -1



    for i in range(0, len(start_states)):
        start = start_states[i]
        goal = goal_states[i]
        # print('print ', Dijkstra(start, goal))

        time_start = time.time()
        cost, expanded_diskstra =  Dijkstra(start, goal)  # replace None, None with the call to your Dijkstra's implementation
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()

            # start = start_states[i]
        # goal = goal_states[i]
        #
        # time_start = time.time()
        # cost, expanded_astar = None, None # replace None, None with the call to your A* implementation
        # time_end = time.time()
        #
        # nodes_expanded_astar.append(expanded_astar)
        # time_astar.append(time_end - time_start)
        #
        # if cost != solution_costs[i]:
        #     print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
        #     print("Start state: ", start)
        #     print("Goal state: ", goal)
        #     print("Solution cost encountered: ", cost)
        #     print("Solution cost expected: ", solution_costs[i])
        #     print()

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)",
                             "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")


if __name__ == "__main__":
    test()
