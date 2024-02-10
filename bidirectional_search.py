from collections import deque
from curses import qiflush
from hashlib import new
import numpy as np
from search_problems import Node, GraphSearchProblem

def bidirectional_search(problem):
    """
        Implement a bidirectional search algorithm that takes instances of SimpleSearchProblem (or its derived
        classes) and provides a valid and optimal path from the initial state to the goal state.

        :param problem: instance of SimpleSearchProblem
        :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
                 num_nodes_expanded: number of nodes expanded by the search
                 max_frontier_size: maximum frontier size during search
        """
    ####
    #   COMPLETE THIS CODE
    ####
    max_frontier_size = 0
    num_nodes_expanded = 0
    path = []

    queue_forward = [[problem.init_state]]
    queue_backward = [[problem.goal_states[0]]]

    visited_forward = []
    visited_backward = []

    adj_list = {}
    for v in problem.V:
        adj_list[v] = []

    for e in problem.E:
        adj_list[e[0]].append(e[1])
        adj_list[e[1]].append(e[0])

    if problem.init_state == problem.goal_states[0]:
        return [problem.init_state]

    #search from beginning node 

    while len(queue_forward) != 0 and len(queue_backward) != 0:
        #print('bkwd:', queue_backward)
        #expand the whole level first
        curr_queue_backward = queue_backward
        for n in range (0, len(curr_queue_backward)):
            path = curr_queue_backward.pop(0)
            num_nodes_expanded += 1
            last_node = path[-1]
            if last_node not in visited_backward:
                for vertex in adj_list[last_node]:
                    new_path = list(path)
                    new_path.append(vertex)
                    queue_backward.append(new_path)
                    if len(queue_backward) + len(queue_forward) > max_frontier_size:
                        max_frontier_size = len(queue_backward) + len(queue_forward)
                    for path_fwd in queue_forward:
                        if vertex == path_fwd[-1]:
                            #print('vertex', vertex, ' intersected!')
                            #print(new_path, path_bkwd)
                            #print('newpath', new_path, 'path_bk', path_bkwd)
                            path_bkwd = new_path[::-1]
                            join_idx = path_bkwd.index(path_fwd[-1])
                            path_bkwd = path_bkwd[join_idx+1:]
                            result = path_fwd + path_bkwd
                            return result, num_nodes_expanded, max_frontier_size
                visited_backward.append(last_node)

        #print('fwd:', queue_forward)
        curr_queue_forward = queue_forward
        for n in range (0, len(curr_queue_forward)):
            path = curr_queue_forward.pop(0)
            num_nodes_expanded += 1
            last_node = path[-1]
            if last_node not in visited_forward:
                for vertex in adj_list[last_node]:
                    new_path = list(path)
                    new_path.append(vertex)
                    queue_forward.append(new_path)
                    if len(queue_backward) + len(queue_forward) > max_frontier_size:
                        max_frontier_size = len(queue_backward) + len(queue_forward)
                    for path_bkwd in queue_backward:
                        if vertex == path_bkwd[-1]:
                            #print('vertex', vertex, ' intersected!')
                            #print(new_path, path_bkwd)
                            #print('newpath', new_path, 'path_bk', path_bkwd)
                            path_bkwd = path_bkwd[::-1]
                            join_idx = path_bkwd.index(new_path[-1])
                            path_bkwd = path_bkwd[join_idx+1:]
                            result = new_path + path_bkwd
                            return result, num_nodes_expanded, max_frontier_size
                visited_forward.append(last_node)

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

    goal_states = [3]
    init_state = 0
    E = np.array([[0, 1],
                [1, 2],
                [2, 3],
                [3, 4],
                [4, 0],
                [1, 5],
                [5, 6],
                [6, 7],
                [7, 8],
                [8, 9],
                [9, 5]])
    goal_states = [8]
    init_state = 2
    E = np.array([[0, 1],
                [1, 2],
                [2, 3],
                [3, 7],
                [7, 8],
                [4, 5],
                [5, 6]])
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    #path, num_nodes_expanded, max_frontier_size = breadth_first_search(problem)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)

    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    #print(path)

    # Use stanford_large_network_facebook_combined.txt to make your own test instances
    #E = np.loadtxt('../datasets/stanford_large_network_facebook_combined.txt', dtype=int)
    E = np.loadtxt('stanford_large_network_facebook_combined.txt', dtype=int)
    V = np.unique(E)
    goal_states = [349]
    init_state = 0
    problem = GraphSearchProblem(goal_states, init_state, V, E)
    path, num_nodes_expanded, max_frontier_size = bidirectional_search(problem)
    correct = problem.check_graph_solution(path)
    print("Solution is correct: {:}".format(correct))
    #print(path)

    # Be sure to compare with breadth_first_search!