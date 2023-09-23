import time
from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq


def main():
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

    # print('Start ', start_states)

    def Dijkstra(start, goal):
        # mapFunc = Map(gridded_map)

        OpenArr = [(0, start.get_x(), start.get_y())]
        ClosedArr = dict()
        ClosedArr[State(start.get_x(), start.get_y()).state_hash()] = start, 0
        # print('goal', goal)

        while (OpenArr) != 0:
            n = heapq.heappop(OpenArr)

            nx = n[1]
            ny = n[2]

            if State(goal.get_x(), goal.get_y()) == State(nx, ny):
                return n[0], len(OpenArr)

            for n_ in gridded_map.successors(State(nx, ny)):
                n_x = n_.get_x()
                n_y = n_.get_y()


                if State(n_x, n_y).state_hash() not in ClosedArr:
                    heapq.heappush(OpenArr, (State(nx, ny).get_g() + gridded_map.cost(n_x, n_y), n_.get_x(), n_.get_y()))

                    ClosedArr[State(n_x, n_y).state_hash()] = n_, State(n_x, n_y).get_g() + gridded_map.cost(n_x, n_y)
                    State(n_x, n_y).set_g(State(n_x, n_y).get_g() + gridded_map.cost(n_x, n_y))
                    # print(State(n_x, n_y).get_g())


                if State(n_x, n_y).state_hash() in ClosedArr and (State(n_x, n_y).get_g() + gridded_map.cost(n_x, n_y)) < ClosedArr[State(n_x, n_y).state_hash()][1]:
                    # update cost in both                               #TODO

                    ClosedArr[State(n_x, n_y).state_hash()] = (n_, State(n).get_g() + gridded_map.cost(n_))

                    index = OpenArr.index(State(nx, ny).get_g() + gridded_map.cost(n_x, n_y), n_)
                    OpenArr[index] = (State(nx, ny).get_g() + gridded_map.cost(n_x, n_y), n_.get_x(), n_.get_y())

            heapq.heapify(OpenArr)
        return -1


    for i in range(0, len(start_states)):
        start = start_states[i]
        goal = goal_states[i]

        time_start = time.time()
        cost, expanded_diskstra = Dijkstra(start, goal)  # replace None, None with the call to your Dijkstra's implementation
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
    main()
