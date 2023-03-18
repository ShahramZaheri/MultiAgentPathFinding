import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location, move


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        # x = range(4,1, -1)
        # for item in x:
        #     print(item)


        start_time = timer.time()
        result = []
        
        # constraints = [{'timestep': 3, 'agent': 0,'loc': [(1,3)], 'positive': True}]
        constraints =[]

        for i in range(self.num_of_agents):  # Find path for each agent
            
            
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            
            result.append(path)
            # print(constraints)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################
            # adding negative vertex constraints to all future agents
            for timeStep in range(len(path)):
                for agent in range(i+1, self.num_of_agents):
                    vertex_constrain = dict()
                    vertex_constrain['agent'] = agent
                    vertex_constrain['loc'] = [get_location(path, timeStep)]
                    vertex_constrain['timestep'] = timeStep
                    vertex_constrain['positive'] = False
                    constraints.append(vertex_constrain)
            # adding edge constraints to all future agents
            # if current agent moves from loc_1 to loc_2 at timestep t, no other agent
            # can move from loc_2 to loc_1 at timestep t.
            for timeStep in range(len(path)):
                for agent in range(i+1, self.num_of_agents):
                    edge_constrain = dict()
                    edge_constrain['loc'] = [get_location(path, timeStep), get_location(path, timeStep-1)]
                    edge_constrain['agent'] = agent
                    edge_constrain['timestep'] = timeStep
                    edge_constrain['positive'] = False
                    constraints.append(edge_constrain)
            
            # task 2.3 once an agent reaches its goal, block that cell for all future agents
            self.my_map[path[-1][0]][path[-1][1] ] = True
            


        # during the program we block all the goal locations on the map.
        # to display the original map, we need to revert these changes.
        for goal in self.goals:
            self.my_map[goal[0]][goal[1]] = False
            
        
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
