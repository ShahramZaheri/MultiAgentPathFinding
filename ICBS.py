import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, list_of_shortest_paths
import copy


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    upper_bound_of_timesteps = max(len(path1), len(path2))
    for timestep in range(upper_bound_of_timesteps):
        if (get_location(path1, timestep) == get_location(path2,timestep)):
            # this is vertex collision
            return({'timestep':timestep, 'loc':[get_location(path1,timestep)]})
        if ((get_location(path1, timestep) == get_location(path2,timestep-1)) and (get_location(path1, timestep-1) == get_location(path2,timestep))):
            # this is edge collision
            return({'timestep':timestep, 'loc':[get_location(path1,timestep-1), get_location(path1,timestep)]})
    return None



def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    first_collisions = []
    for i in range(len(paths) -1):
        for j in range(i+1, len(paths)):
            first_collision = detect_collision(paths[i], paths[j])
            # print(first_collision)
            
            if ( first_collision != None):
                # a collision found between agent i and j
                first_collision['a1'] = i
                first_collision['a2'] = j
                first_collisions.append(first_collision)
    return first_collisions



def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints = []
    if (len (collision['loc'])== 1):
        # it's a vertex collision
        constraints.append({'timestep': collision['timestep'], 'loc': collision['loc'], 'agent':collision['a1'], 'positive':False})
        constraints.append({'timestep': collision['timestep'], 'loc': collision['loc'], 'agent':collision['a2'], 'positive':False})
    elif (len (collision['loc'])== 2):
        # it's a edge collision
        constraints.append({'timestep': collision['timestep'], 'loc':[collision['loc'][0], collision['loc'][1]] , 'agent':collision['a1'], 'positive':False})
        constraints.append({'timestep': collision['timestep'], 'loc': [collision['loc'][1], collision['loc'][0]], 'agent':collision['a2'], 'positive':False})

    return constraints

def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    constraints = []
    random_int = random.randint(0,1)
    random_int = 0

    if (random_int == 0):
        random_agent = 'a1'
    else:
        random_agent = 'a2'

    if (len (collision['loc'])== 1):
        # it's a vertex collision
        constraints.append({'timestep': collision['timestep'], 'loc': collision['loc'], 'agent':collision[random_agent], 'positive':False})
        constraints.append({'timestep': collision['timestep'], 'loc': collision['loc'], 'agent':collision[random_agent], 'positive':True})
    elif (len (collision['loc'])== 2):
        # it's a edge collision
        constraints.append({'timestep': collision['timestep'], 'loc':[collision['loc'][0], collision['loc'][1]] , 'agent':collision[random_agent], 'positive':False})
        constraints.append({'timestep': collision['timestep'], 'loc': [collision['loc'][0], collision['loc'][1]], 'agent':collision[random_agent], 'positive':True})

    return constraints


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class ICBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        

        # print("hello from ICBS")
        all_shortest_paths = list_of_shortest_paths(self.my_map, self.starts[1], self.goals[1], self.heuristics[1], 1, [{'timestep': 2, 'agent': 1,'loc': [(1,5)], 'positive': True}])
        print("list of all shortest paths:")
        print(all_shortest_paths)

    # def push_node(self, node):
    #     heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
    #     # print("Generate node {}".format(self.num_of_generated))
    #     self.num_of_generated += 1

    # def pop_node(self):
    #     _, _, id, node = heapq.heappop(self.open_list)
    #     # print("Expand node {}".format(id))
    #     self.num_of_expanded += 1
    #     return node

    # def find_solution(self, disjoint=True):
    #     """ Finds paths for all agents from their start locations to their goal locations

    #     disjoint    - use disjoint splitting or not
    #     """       
    #     self.start_time = timer.time()

    #     # Generate the root node
    #     # constraints   - list of constraints
    #     # paths         - list of paths, one for each agent
    #     #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
    #     # collisions     - list of collisions in paths
    #     root = {'cost': 0,
    #             'constraints': [],
    #             'paths': [],
    #             'collisions': []}
    #     for i in range(self.num_of_agents):  # Find initial path for each agent
    #         path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
    #                       i, root['constraints'])
    #         if path is None:
    #             raise BaseException('No solutions')
    #         root['paths'].append(path)

    #     root['cost'] = get_sum_of_cost(root['paths'])
    #     root['collisions'] = detect_collisions(root['paths'])
    #     self.push_node(root)

    #     # Task 3.1: Testing
    #     # print(root['collisions'])

    #     # Task 3.2: Testing
    #     # for collision in root['collisions']:
    #     #     print(standard_splitting(collision))

    #     ##############################
    #     # Task 3.3: High-Level Search
    #     #           Repeat the following as long as the open list is not empty:
    #     #             1. Get the next node from the open list (you can use self.pop_node()
    #     #             2. If this node has no collision, return solution
    #     #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
    #     #                standard_splitting function). Add a new child node to your open list for each constraint
    #     #           Ensure to create a copy of any objects that your child nodes might inherit
        
    #     number_of_iteration =0
    #     max_number_of_iterations = 300000
    #     while (len(self.open_list) != 0 and number_of_iteration < max_number_of_iterations):
    
    #         popped_node = copy.deepcopy(self.pop_node())
    #         # print(popped_node)
    #         # print("===========================================")
    #         number_of_iteration +=1
           
    #         if (len(popped_node['collisions']) == 0):
    #             # this node is a goal node, return its paths
    #             # print("number of nodes expanded = {}".format(number_of_iteration))
    #             # print(popped_node)
    #             for i in range(len(popped_node['paths'])):
    #                 print("agent {}".format(i))
    #                 print("path = {}".format(popped_node['paths'][i]))
    #             self.print_results(popped_node)
    #             return popped_node['paths']

            
    #         my_collision = popped_node['collisions'][-1]

    #         if disjoint:
    #             my_constraints = disjoint_splitting(my_collision)
    #         else:
    #             my_constraints = standard_splitting(my_collision)
    #         # my constraints has two constraints that should be added to the child nodes
    #         prune_the_child_node = False
    #         # print("======================================")
    #         # print("my constraints = {}".format(my_constraints))
            
    #         for constraint in my_constraints:
    #             # print(constraint)
    #             new_node = {'cost': 0,
    #                         'constraints': [],
    #                         'paths': [],
    #                         'collisions': []
    #                         }
    #             for item in popped_node['constraints']:
    #                 new_node['constraints'].append(item)
    #             new_node['constraints'].append(constraint)
                
    #             for path in popped_node['paths']:
    #                 new_node['paths'].append(path)
                
    #             if (constraint['positive']):
    #                 # positive constraint for an agent means negative constraint for all other agents
    #                 # for agent_id in range(self.num_of_agents):
    #                 #     if agent_id == constraint['agent']:
    #                 #         continue
    #                 #     else:
    #                 #         if len(constraint['loc']) ==1:
    #                 #             new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':constraint['loc'] , 'agent':agent_id, 'positive':False})
    #                 #         else:
    #                 #             new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':[constraint['loc'][1], constraint['loc'][0]], 'agent':agent_id, 'positive':False})
    #                 #             new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':[constraint['loc'][0], constraint['loc'][1]], 'agent':agent_id, 'positive':False})
    #                 #             new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':constraint['loc'] , 'agent':agent_id, 'positive':False})
                    
                    
    #                 # paths that have a conflict with the positive constraint should be updated
    #                 agents_violating_positive_constraint = paths_violate_constraint(constraint, new_node['paths'])
    #                 # print("list of violating agents = {}".format(agents_violating_positive_constraint))
    #                 for agent in agents_violating_positive_constraint:
                        
    #                     if len(constraint['loc']) ==1:
    #                         new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':constraint['loc'] , 'agent':agent, 'positive':False})
    #                     else:
    #                         new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':[constraint['loc'][1], constraint['loc'][0]], 'agent':agent, 'positive':False})
    #                         # new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':[constraint['loc'][0], constraint['loc'][1]], 'agent':agent, 'positive':False})
    #                         # new_node['constraints'].append({'timestep': constraint['timestep'], 'loc':constraint['loc'] , 'agent':agent, 'positive':False})
                        
    #                     new_path_for_agent_violating_positive_constraint = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, new_node['constraints'])
    #                     if new_path_for_agent_violating_positive_constraint is None:
    #                         prune_the_child_node = True
    #                     else:
    #                         new_node['paths'][agent] = new_path_for_agent_violating_positive_constraint
    #             # else:
    #             #     my_agent = constraint['agent']
    #             #     my_path = a_star(self.my_map, self.starts[my_agent], self.goals[my_agent], self.heuristics[my_agent], my_agent, new_node['constraints'])
    #             #     if my_path is None:
    #             #         prune_the_child_node = True
    #             #     else:
    #             #         new_node['paths'][my_agent] = my_path
    #             my_agent = constraint['agent']
    #             my_path = a_star(self.my_map, self.starts[my_agent], self.goals[my_agent], self.heuristics[my_agent], my_agent, new_node['constraints'])
    #             if my_path is None:
    #                 prune_the_child_node = True
    #             else:
    #                 new_node['paths'][my_agent] = my_path

                
    #             if (prune_the_child_node):
    #                 # print("child pruned ====================================== ")
    #                 continue
    #                 # new node needs to be pruned so don't add it to the open list
    #             else:
    #                 # print("child node constraints ={}".format(new_node['constraints']))
    #                 new_node['collisions'] = detect_collisions(new_node['paths'])
    #                 new_node['cost'] = get_sum_of_cost(new_node['paths'])
    #                 self.push_node(new_node)

               
    #     print('No solution')
    #     return None

    #     # self.print_results(root)
    #     # return root['paths']


    # def print_results(self, node):
    #     print("\n Found a solution! \n")
    #     CPU_time = timer.time() - self.start_time
    #     print("CPU time (s):    {:.2f}".format(CPU_time))
    #     print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
    #     print("Expanded nodes:  {}".format(self.num_of_expanded))
    #     print("Generated nodes: {}".format(self.num_of_generated))
