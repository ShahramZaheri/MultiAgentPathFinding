# Multi-Agent Path Finding
I did this project in fall 2021 as part of the course porject for [CMPT-417](http://www.sfu.ca/outlines.html?2022/summer/cmpt/417/d100). Instructor of the course was [Dr. Hang Ma](https://www.cs.sfu.ca/~hangma/). The code for visualization of the program was provided by the instructor.
## Introduction
Multi-Agent Path Finding (MAPF) is about finding shortest collision-free paths for a set of agents to take the agents from their start locations to their goal locations. Shortest path(s) for a single agent can be found using different seach algorithms includeing Breadth First Search (BFS) or A<sup>*</sup> search. However, these search algoithms cannot be generalized for multi agents. For example, if we assume that there are five possible movements (i.e. left, right, up, down, and no movement) for an agent in a given location, for two agents, there will be 25 possible movements. Therefor, the tree for the joint space search will be extremely large and inefficient to traverse.
Conflict Based Search (CBS) is one of the approaches to solve multi-agent path finding problem. In this approach, shortest path for each agent is found independently. If the paths don't collide, the MAPF problem is solved. However, if there is a collision between two paths, the collision is added as a negative constraint to the sarch problem of the agents. For example, if agent a and b collide at location x and at time t, the individual search problems are modified such that either agent a or agent b should not be at location x and at time t. 
## How to run the program:
1. Go to env\Scripts and activate the environment by command .\activate
2. Now navigate back to MultiAgentPathFinding directory and enter following command <br/>
'python .\run_experiments.py --instance .\instances\test_20.txt --solver CBS'
3. As we can see, the above command takes two inputs: name of the experiment from instances folder to run (e.g. test_20.txt) and type of the solver (e.g. CBS)
