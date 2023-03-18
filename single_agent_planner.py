import heapq
from os import path
import random

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    constraints_table = []
    for constraint in constraints:
        if constraint['agent'] == agent:
            constraints_table.append({'timestep': constraint['timestep'],'loc': constraint['loc'], 'positive': constraint['positive']})
    # print("constraint table for agent = {}".format(agent))
    # print(constraints_table)
    return constraints_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def can_stay_at_goal(curr_loc, curr_time, constraint_table):
    for constraint in constraint_table:
        # constrain_table is a list of dictionaries
        # for time in constraint:
        #     if (time >= curr_time) and (constraint[time]==[curr_loc]):
        #         return False
        if (constraint['timestep'] > curr_time and constraint['loc'] == [curr_loc] and constraint['positive'] == False):
            # there is a negative constraint for this location in future
            return False
    return True 
def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    for constraint in constraint_table:
        if ( (constraint['timestep'] == next_time) and (not constraint['positive']) and (constraint['loc'] == [next_loc] or constraint['loc'] == [curr_loc, next_loc])):
            # given time and location violates a negative (vertex or edge) constraint
            return True
        if ( (constraint['timestep'] == next_time) and (constraint['positive']) and (constraint['loc'] != [next_loc] and constraint['loc'] != [curr_loc, next_loc])):
            # given time and location violates a positive (vertex or edge) constraint
            return True
    # the given timestep and location do not violate any negative or positive constraints, so return False since the given condition is not constrained
    return False
        
def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['timestep'], node['loc'], random.randint(1,1000), node))


def pop_node(open_list):
    _,_, _, _,_, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    if (n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']):
        return True
    # elif (n1['g_val'] + n1['h_val'] == n2['g_val'] + n2['h_val']) and n1['timestep'] < n2['timestep']:
    #     return True
    # return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    # if (goal_is_reachable_from_start(my_map, start_loc, goal_loc) == False):
    #     return None

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    # print(my_map)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    
    max_time_step = len(constraint_table)
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if (not my_map[i][j]):
                max_time_step +=1
    max_time_step +=1
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    # added 'time' to closed list
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['timestep'] > max_time_step:
            return None
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if (curr['loc'] == goal_loc) and can_stay_at_goal(curr['loc'], curr['timestep'], constraint_table):
            return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # making sure that child node is not out of board
            row_of_child = child_loc[0]
            col_of_child = child_loc[1]
            if (row_of_child <0 or row_of_child>=len(my_map) or col_of_child < 0 or col_of_child >= len(my_map[0])):
                continue
            if (is_constrained(curr['loc'], child_loc, curr['timestep']+1, constraint_table) or my_map[child_loc[0]][child_loc[1]]):
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,

                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
# ===================================================================== group project code starts from here ===========================================================

def list_of_shortest_paths(my_map, start_loc, goal_loc, h_values, agent, constraints):
    # this is modified version of a_star function defined above.
    # this function returns a list of all the shortest paths between start_loc and goal_loc
    list_of_shortest_paths = []
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent)
    
    max_time_step = len(constraint_table)
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if (not my_map[i][j]):
                max_time_step +=1
    max_time_step +=1
    
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        # print("====================================================")
        # print("popped node")
        # print("f_val = {}, h_val = {}, timesteip = {}, location = {}".format(curr['g_val']+curr['h_val'], curr['h_val'],curr['timestep'], curr['loc']))
        # print("=====================================================")
        if curr['timestep'] > max_time_step:
            return None
       
        if (curr['loc'] == goal_loc) and can_stay_at_goal(curr['loc'], curr['timestep'], constraint_table):
            if (len(list_of_shortest_paths) == 0):
                # this is the first shortest path, keep the timestep
                earliest_goal_timestep = curr['timestep']
            if curr['timestep'] == earliest_goal_timestep:
                shortest_path = get_path(curr)
                # print(shortest_path)
                list_of_shortest_paths.append(shortest_path)
                # continue
            elif curr['timestep'] > earliest_goal_timestep:
                # print("====================== ")
                # print("len of open list = {}".format(len(open_list)))
                x = get_path(curr)
                print(x)
                # paths longer that the shortest path started, retun the list of the shortest path
                return list_of_shortest_paths
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            # making sure that child node is not out of board
            row_of_child = child_loc[0]
            col_of_child = child_loc[1]
            if (row_of_child <0 or row_of_child>=len(my_map) or col_of_child < 0 or col_of_child >= len(my_map[0])):
                continue
            if (is_constrained(curr['loc'], child_loc, curr['timestep']+1, constraint_table) or my_map[child_loc[0]][child_loc[1]]):
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            
            # print("child f_val = {}, h_val = {}, timesteip = {}, location = {}".format(child['g_val']+child['h_val'], child['h_val'],child['timestep'], child['loc']))
            push_node(open_list, child)
            

            # if (child['loc'], child['timestep']) in closed_list:
            #     existing_node = closed_list[(child['loc'], child['timestep'])]
            #     if (child['g_val'] + child['h_val'] <= existing_node['g_val'] + existing_node['h_val']):
            #         closed_list[(child['loc'], child['timestep'])] = child
            #         print(child)
            #         push_node(open_list, child)
            # else:
            #     closed_list[(child['loc'], child['timestep'])] = child
            #     push_node(open_list, child)
            #     print(child)
    
    return None

