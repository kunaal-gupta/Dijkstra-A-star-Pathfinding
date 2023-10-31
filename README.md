
## Dijkstra-A-star-Pathfinding 

## Overview 

In this assignment you will implement Dijkstra’s algorithm and A* for solving pathfinding problems on video
game maps. We will consider a grid environment where each action in the four cardinal directions (north,
south, east, and west) has the cost of 1.0 and each action in one of the four diagonal directions has the cost
of 1.5. Each search problem is defined by a video game map, a start location, and a goal location. The
assignment package available on eClass includes a large number of maps from movingai.com, but you will
use a single map in our experiments; feel free to explore other maps if you like.

Most of the code you need is already implemented in the assignment package. In the next section, we detail
some of the key functions you will use from the starter code. You can reimplement all these functions if you
prefer, their use isn’t mandatory. The assignment must be implemented in Python, however.

![1695912395935](https://github.com/kunaal-gupta/Dijkstra-A-star-Pathfinding/assets/87749508/893ff006-4319-43b2-84a0-7a70f66d6f2c)
![1695912411225](https://github.com/kunaal-gupta/Dijkstra-A-star-Pathfinding/assets/87749508/b0964908-0f53-4faa-a6b1-202f78bd0681)

## Part 1: Djiktra's Path Finding

Implement Dijkstra’s algorithm and call your implementation in the line marked with the comment “replace
None, None with the call to your Dijkstra’s implementation” in main.py. The implementation must be
correct, i.e., it must find an optimal solution for the search problems. The algorithm must return the
solution cost and the number of nodes it expands to find a solution. If the problem has no solution, it must
return 􀀀1 for the cost. There is no need to recover the optimal path the algorithm encounters, but only
report the cost and number of expansions.

The implementation must be efficient, i.e., it should use the correct data structures. You can test the
correctness of your implementation of Dijkstra’s algorithm by running python3 main.py. You may also use
the plotting function of the Map class to visualize the result of your search.

You can implement the algorithm as a function or as a class, whichever is more convenient for you. Your
implementation can be in a new file, in main.py, or in any other file you prefer.


##  Part 2: A* Path Finding

Implement A* and call your implementation in the line marked with the comment “replace None, None with
the call to your A* implementation” in main.py. We will use the Octile distance with our implementation
of A*. Octile distance is a version of the Manhattan distance function we have seen in class that accounts
for diagonal moves. Intuitively, if we are considering a map free of obstacles, the agent will perform as many
4
diagonal moves as possible because a diagonal move allows one to progress in both the x and y coordinates
toward the goal. Let change in x and change in y be the absolute differences in distance in the x-axis and in the y-axis,
respectively, between the evaluated state and the goal state. The maximum number of diagonal moves
we can perform is given by min(change in x; change in y) and each move costs 1:5; the values that cannot be corrected with
diagonal moves are corrected with regular cardinal moves, where each move costs 1:0, and there are absolute(change in x - change in y)
of them. Octile distance can be written as

h(s) = 1.5 * min(change in x; change in y) + absolute(change in x - change in y);
The Octile distance is consistent and thus admissible. Since the heuristic is consistent, you do not have to
implement the re-expansion of nodes we discussed in class.

You can implement the algorithm as a function or as a class, whichever is more convenient for you. Your
implementation can be in a new file, in main.py, or in any other file you prefer. Similarly to the A*
implementation, you can implement the Octile distance anywhere you prefer in the code.


## How to Run Starter Code
Follow the steps below to run the starter code (instructions are for Mac and Linux). 
 - Install Python 3.
 - It is usually a good idea to create a virtual environment to install the libraries needed for the assignment.


The virtual environment step is optional.
  - virtualenv -p python3 venv
  - source venv/bin/activate

When you are done working with the virtual environment you can deactivate it by typing deactivate.
Run pip install -r requirements.txt to install the libraries specified in requirements.txt.

## Authors

- [@kunaal-gupta](https://github.com/kunaal-gupta/)


## Contributing

Contributions are always welcome!

Please connect with me for more details  kunaalgupta@hotmail.com.


## About Me
I'm a full-stack software engineer who aspires to develop and deliver sustainable technologies in an Agile environment while ensuring optimal functionality, quality, and reliability.

Languages: Python - Django, Pandas, NumPy, OpenCV, Matplotlib, SciKit, Tkinter | C++ | JS, Ajax, jQuery, HTML5/CSS | Blockchain, Solidity, ETH, | MongoDB, MySQL, SQLite | Testing |

Tools: PyCharm, Visual Studio Code, JIRA, GitHub, Truffle, Metamask, Ganache, Remix IDE, MS Suite

## Installation

In your terminal, install following python libraries.

```bash
pip install Map
```
```bash
pip install time
```
```bash
pip install getopt
```
```bash
pip install sys
```
```bash
pip install heapq
```

## Tech Stack

**Library:** Heapq, Sys, Time, Getopt, Map

**Language** Python

**Tool:** PyCharm

