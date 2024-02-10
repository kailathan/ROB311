from collections import deque
from hashlib import new
import numpy as np
from search_problems import Node, GraphSearchProblem

def breadth_first_search(problem):
    """
    Implement a simple breadth-first search algorithm that takes instances of SimpleSearchProblem (or its derived
    classes) and provides a valid and optimal path from the initial state to the goal state. Useful for testing your
    bidirectional and A* search algorithms.

    :param problem: instance of SimpleSearchProblem
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by the search
             max_frontier_size: maximum frontier size during search
    """
    ####
    #   COMPLETE THIS CODE
    ####
    #path
    path = []
    #visited
    visited = []
    #queue
    queue = [[problem.init_state]]
  
    num_nodes_expanded = 0
    max_frontier_size = 0

    #adj list 
    adj_list = {}
    for v in problem.V:
        adj_list[v] = []

    for e in problem.E:
        adj_list[e[0]].append(e[1])
        adj_list[e[1]].append(e[0])

    if problem.init_state == problem.goal_states[0]:
        return [problem.init_state]
    
    while queue:
        path = queue.pop(0)
        last = path[-1]
        if last not in visited:
            #if last node not visited, expand it.
            num_nodes_expanded += 1
            for new_vertex in adj_list[last]:
                new_path = list(path)
                new_path.append(new_vertex)
                queue.append(new_path)
                #update max frontier size
                if len(queue) > max_frontier_size:
                    max_frontier_size = len(queue)
                if new_vertex == problem.goal_states[0]:
                    #print(new_path)
                    return new_path, num_nodes_expanded, max_frontier_size
            visited.append(last)

    return path, num_nodes_expanded, max_frontier_size


if __name__ == '__main__':
    # Simple example
    goal_states = [0]
    init_state = 9
    V = np.arange(0, 10)
    E = np.array([[0, 1],
                  [1, 2],
                  [2, 3],
                  [3, 4],
                  [4, 5],
                  [5, 6],
                  [6, 7],
                  [7, 8],
                  [8, 9],
                  [0, 6],
                  [1, 7],
                  [2, 5],
                  [9, 4]])
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = 0
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)

    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))