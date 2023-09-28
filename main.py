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

    plots = True
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

    def OctileDistance(x, y, goal):
        xDiff = abs(x - goal.get_x())
        yDiff = abs(y - goal.get_y())
        return 1.5 * min(xDiff, yDiff) + abs(xDiff - yDiff)

    def Astar(start, goal):

        OpenArr = [[0, start]]
        ClosedArr = {State(start.get_x(), start.get_y()).state_hash(): [start, 0]}

        while OpenArr:
            node_g, node = heapq.heappop(OpenArr)

            if node == goal:
                return ClosedArr[goal.state_hash()][1], len(ClosedArr)

            for childNode in gridded_map.successors(node):
                child_g = childNode.get_g()
                child_h = OctileDistance(childNode.get_x(), childNode.get_y(), goal)

                if childNode.state_hash() not in ClosedArr:
                    heapq.heappush(OpenArr, [child_g+child_h, childNode])  # OpenList
                    ClosedArr[childNode.state_hash()] = [childNode, child_g+child_h]  # ClosedList

                if childNode.state_hash() in ClosedArr and child_g+child_h < ClosedArr[childNode.state_hash()][1]:
                    ClosedArr[childNode.state_hash()][1] = child_g+child_h
                    heapq.heappush(OpenArr, [child_g+child_h, childNode])
                    childNode.set_g(child_g)

        return -1, -1

    def Dijkstra(start, goal):
        OpenArr = [[0, start]]
        ClosedArr = {start.state_hash(): [start, 0]}

        while OpenArr:
            node_g, node = heapq.heappop(OpenArr)

            if node == goal:
                return ClosedArr[goal.state_hash()][1], len(ClosedArr)

            for childNode in gridded_map.successors(node):
                child_g = childNode.get_g()

                if childNode.state_hash() not in ClosedArr:
                    heapq.heappush(OpenArr, [child_g, childNode])  # OpenList
                    ClosedArr[childNode.state_hash()] = [childNode, child_g]  # ClosedList

                elif child_g < ClosedArr[childNode.state_hash()][1]:
                    ClosedArr[childNode.state_hash()][1] = child_g
                    heapq.heappush(OpenArr, [child_g, childNode])

        return -1, -1

    for i in range(0, len(start_states)):
        start = start_states[i]
        goal = goal_states[i]

        time_start = time.time()
        cost, expanded_diskstra = Dijkstra(start, goal)  # replace None, None with the call to your Dijkstra's
        # implementation
        time_end = time.time()
        nodes_expanded_dijkstra.append(expanded_diskstra)
        time_dijkstra.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by Dijkstra and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])

        else:
            print('Dj Time', time_end - time_start)



        start = start_states[i]
        goal = goal_states[i]

        time_start = time.time()
        cost, expanded_astar = Astar(start, goal) # replace None, None with the call to your A* implementation
        time_end = time.time()

        nodes_expanded_astar.append(expanded_astar)
        time_astar.append(time_end - time_start)

        if cost != solution_costs[i]:
            print("There is a mismatch in the solution cost found by A* and what was expected for the problem:")
            print("Start state: ", start)
            print("Goal state: ", goal)
            print("Solution cost encountered: ", cost)
            print("Solution cost expected: ", solution_costs[i])
            print()
        else:
            print('astar Time', time_end - time_start)

    if plots:
        from search.plot_results import PlotResults
        plotter = PlotResults()
        plotter.plot_results(nodes_expanded_astar, nodes_expanded_dijkstra, "Nodes Expanded (A*)",
                             "Nodes Expanded (Dijkstra)", "nodes_expanded")
        plotter.plot_results(time_astar, time_dijkstra, "Running Time (A*)", "Running Time (Dijkstra)", "running_time")


if __name__ == "__main__":
    main()
