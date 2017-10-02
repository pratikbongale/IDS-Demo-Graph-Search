import sys


# SearchGraph.py
#
# Implementation of iterative deepening search for use in finding optimal routes
# between locations in a graph. In the graph to be searched, nodes have names
# (e.g. city names for a map).
#
# An undirected graph is passed in as a text file (first command line argument).
#
# Usage: python SearchGraph.py graphFile startLocation endLocation
#
# Author: Richard Zanibbi, RIT, Nov. 2011
def read_graph(filename):
    """Read in edges of a graph represented one per line,
	using the format: srcStateName destStateName"""

    print("\n\n========== Loading graph: " + filename + '==================')
    edges = {}

    inFile = open(filename)
    for line in inFile:
        roadInfo = line.split()

        # Skip blank lines, read in contents from non-empty lines.
        if (len(roadInfo) > 0):
            srcCity = roadInfo[0]
            destCity = roadInfo[1]

            if srcCity in edges:
                edges[srcCity] = edges[srcCity] + [destCity]
            else:
                edges[srcCity] = [destCity]

            if destCity in edges:
                edges[destCity] = edges[destCity] + [srcCity]
            else:
                edges[destCity] = [srcCity]

    print("  done.\n")
    return edges


######################################
# Add functions for search, output
# etc. here
######################################

class Problem:
    '''
    Defines the Search problem formulation
    :param: start - Start state
    :param: goal - Goal state
    :param: map - Graph to search using IDS
    '''

    # class variables common to all instances
    start = None
    goal = None
    map = None

    def __init__(self, edges, start, goal):
        Problem.start = start
        Problem.goal = goal
        Problem.map = edges


class Node:
    '''
    Defines a Search Node
    :param: state - Current state (name of town) as str
    :param: parent - Current parent node as Node
    :param: actions - List of actions which can be performed from this state as list of strings(States reachable)
    :param: pathCost - Number of actions required to reach this state as int
    '''

    def __init__(self, state, parent=None, actions=None, pathCost=None):
        self.state = state
        self.parent = parent
        self.actions = actions
        self.pathCost = pathCost

# Dictionary to add all the visited states
exploredSet = {}

def iterativeDeepeningSearch(problem):
    '''
    Implementation of Iterative-Deepening-Search to run until all the states are visited or a solution is found
    :param problem: The search problem compilation(start state, goal state, map)
    :return: result-The goal-state node if found.
    :return: status-The status of this execution ('success', 'cutoff', 'failure')
    '''
    depth = 0   # initial depth

    status = 'init'
    while status is not "success" and status is not "failure":
        print('\nDepth : ' + str(depth))

        # call Depth-Limited-Search with given depth
        result, status = depthLimitedSearch(problem, depth)

        depth += 1
        exploredSet.clear()

    return result, status

def depthLimitedSearch(problem, limit):
    '''
    Recursive implementation of Depth-Limited-Search algorithm
    :param problem: Consisting of start state, goal state and the map
    :param limit: Depth limit for this search
    :return: result-The goal node if found.
    :return: status-The status of this execution ('success', 'cutoff', 'failure')
    '''

    initialNodeState = problem.start    # type string
    initialNode = Node(initialNodeState, parent=None, actions=problem.map[initialNodeState], pathCost=0) # type Node
    return recursiveDLS(initialNode, problem, limit)    # helper function


def recursiveDLS(node, problem, limit):
    '''
    A helper function to recursively complete a depth limited search with max depth = limit
    :param node: The node from where we begin our search
    :param problem: The search problem compilation(start state, goal state, map)
    :param limit: The maximum depth of recursion
    :return: result-The goal node if found.
    :return: status-The status of this execution ('success', 'cutoff', 'failure')
    '''

    if problem.goal == node.state:
        print(node.state)
        return node, "success"
    elif limit == 0:
        exploredSet[node.state] = node
        print(node.state)
        return node, "cutoff"
    else:
        cutoff_occured = False
        exploredSet[node.state] = node
        print(node.state)

        for child in node.actions:

            # Explore the child node if it has not been visited before
            if child not in exploredSet.keys():
                print('\t' * (node.pathCost+1), end='')
                # create a child node
                childNode = Node(state=child, parent=node, actions=problem.map[child], pathCost=node.pathCost+1)
                result, status = recursiveDLS(childNode, problem, limit-1)

            else:
                # the child node is already explored
                # check and update the path cost of the child in explored set if needed
                if exploredSet[child].pathCost > node.pathCost+1:
                    exploredSet[child].pathCost = node.pathCost+1
                    exploredSet[child].parent = node

                    # exploring child node again to further update its children to have new costs
                    print('\t' * (node.pathCost+1), end='')
                    result, status = recursiveDLS(exploredSet[child], problem, limit - 1)
                else:
                    continue

            if status == "cutoff":
                cutoff_occured = True
                # try remaining branches else return failure
            elif status == "success":
                return result, "success"

        if cutoff_occured:
            return node, "cutoff"
        else:
            return node, "failure"


#########################
# Main program
#########################
def main():
    if len(sys.argv) != 4:
        print('Usage: python SearchGraph.py graphFilename startNode goalNode')
        return
    else:
        # Create a dictionary (i.e. associative array, implemented as a hash
        # table) for edges in the map file, and define start and end states for
        # the search. Each dictionary entry key is a string for a location,
        # associated with a list of strings for the adjacent states (cities) in
        # the state space.

        edges = read_graph(sys.argv[1])
        start = sys.argv[2]
        goal = sys.argv[3]

        # Comment out the following lines to hide the graph description.
        # print("-- Adjacent Cities (Transition/Successor Fn) ------------------------")
        # for location in edges.keys():
        #     s = '  ' + location + ':\n     '
        #     s = s + str(edges[location])
        #     print(s)

        if not start in edges.keys():
            print("Start location is not in the graph.")
        else:

            # edges contains the whole map
            searchProblem = Problem(edges, start, goal)

            print('Part a')
            print('-- States Visited ----------------')
            result, status = iterativeDeepeningSearch(searchProblem)

            print('\nPart b')
            if status == 'success':
                print('--  Solution for: ' + start + ' to ' + goal + '-------------------')

                sol = []
                while result is not None:
                    sol.append(result.state)
                    result = result.parent

                for state in reversed(sol):
                    print(state + '     ', end='')
            else:
                print('FAILED SEARCH')
            print('')


# Execute the main program.
main()
