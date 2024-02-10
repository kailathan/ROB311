import queue
import numpy as np
from search_problems import Node, GridSearchProblem, get_random_grid_problem

def a_star_search(problem):
    """
    Uses the A* algorithm to solve an instance of GridSearchProblem. Use the methods of GridSearchProblem along with
    structures and functions from the allowed imports (see above) to implement A*.

    :param problem: an instance of GridSearchProblem to solve
    :return: path: a list of states (ints) describing the path from problem.init_state to problem.goal_state[0]
             num_nodes_expanded: number of nodes expanded by your search
             max_frontier_size: maximum frontier size during search
    """
    ####
    #   COMPLETE THIS CODE
    ####
    num_nodes_expanded = 0
    max_frontier_size = 0
    path = []

    #frontier queue to store all paths we are visiting
    frontier = queue.PriorityQueue()

    #starting node and starting cost gets added as first pair in our frontier, also 
    #fist cost is 0, store in costs list
    #initialize visited set to keep track of all visited states
    starting_node = Node(None, problem.init_state, None, 0)
    starting_cost = problem.heuristic(problem.init_state)
    frontier.put((starting_cost, starting_node))
    costs = {problem.init_state: 0}
    visited = set()

    #grid_map = list(zip(*problem.grid_map))

    while not frontier.empty():
        #pop current cell from frontier
        current_cost, current_cell = frontier.get()

        max_frontier_size = max(max_frontier_size, frontier.qsize())
        num_nodes_expanded += 1

        if problem.goal_test(current_cell.state):
            path = problem.trace_path(current_cell)
            return path, num_nodes_expanded, max_frontier_size
        visited.add(current_cell.state)
#iterate through all transitions from current cell state
        for transition in problem.get_actions(current_cell.state):
            #get all children and store/update the cost to get to child state using current state cost + heuristic
            child = problem.get_child_node(current_cell, transition)
            h = problem.heuristic(child.state)
            cost =  costs[current_cell.state] + problem.action_cost(current_cell.state, transition, child.state)
            estimate = cost + h

            if child.state not in visited:
                if child.state not in costs or cost < costs[child.state]:
                    costs[child.state] = cost
                    child.parent = current_cell
                    frontier.put((estimate, child))
            #repeat for all other nodes in the frontier after adding children

    return [], num_nodes_expanded, max_frontier_size

def search_phase_transition():
    """
    Simply fill in the prob. of occupancy values for the 'phase transition' and peak nodes expanded within 0.05. You do
    NOT need to submit your code that determines the values here: that should be computed on your own machine. Simply
    fill in the values!

    :return: tuple containing (transition_start_probability, transition_end_probability, peak_probability)
    """
    ####
    #   REPLACE THESE VALUES
    ####
    transition_start_probability = 0.3
    transition_end_probability = 0.5
    peak_nodes_expanded_probability = 0.4
    return transition_start_probability, transition_end_probability, peak_nodes_expanded_probability


if __name__ == '__main__':
    # Test your code here!
    # Create a random instance of GridSearchProblem
    p_occ = 0.25
    M = 10
    N = 10
    problem = get_random_grid_problem(p_occ, M, N)
    # Solve it
    path, num_nodes_expanded, max_frontier_size = a_star_search(problem)
    # Check the result
    correct = problem.check_solution(path)
    print("Solution is correct: {:}".format(correct))
    # Plot the result
    #print('grid map', problem.grid_map)
    #print(problem.init_state)
    #print(problem.goal_states)
    #problem.plot_solution(path)
    
    # Experiment and compare with BFS