from search.algorithms import State
from search.map import Map
import getopt
import sys
import heapq
import time


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

    def Dijkstra(start, goal, transitionFunc):
        OpenArr = [start]  # Open Array

        ClosedArr = dict()  # Closed Array
        ClosedArr[State(goal.get_x(), goal.get_y()).state_hash()] = goal

        while OpenArr:
            node = heapq.heappop(OpenArr)

            # If the current state is the goal state, return the solution cost and nodes expanded
            if node == goal:
                return ClosedArr[node.state_hash()].get_g(), len(ClosedArr)

            # Mark the current state as explored
            ClosedArr[node.state_hash()] = node

            for childNode in transitionFunc.successors(node):
                child_x, child_y, child_hash = childNode.get_x(), childNode.get_y(), childNode.state_hash()

                childNode.set_cost(childNode.get_g())

                if ClosedArr.get(childNode.state_hash()) is None:
                    heapq.heappush(OpenArr, childNode)
                    ClosedArr[child_hash] = childNode

                elif childNode.get_g() < ClosedArr[child_hash].get_g():

                    childNode.set_cost(childNode.get_cost)
                    childNode.set_g(childNode.get_g())

        return -1, -1



    for i in range(0, len(start_states)):
        start = start_states[i]
        goal = goal_states[i]
        time_start = time.time()
        cost, expanded_diskstra = Dijkstra(start, goal, gridded_map)

        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            # print(Arr)
            print()
        else:
            print('success')

    if plots:
        from search.plot_results import PlotResults
        import matplotlib.pyplot as plt
        import matplotlib
        matplotlib.use("TkAgg")

        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)",
                             "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")
        plt.show()


if __name__ == "__main__":
    test()
