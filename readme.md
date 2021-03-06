# Reactive Agents, CS-430 Assignement 3

![screenshot](screenshot.png)
Implementation of a planning deliberative agent with graph-search algorithms.

## Tasks from the problem definition :
* **DONE** Choose an adequate reprensentation of **states**, **transitions** and **goals** (final states), such that BFS or ASTAR will be able to find an optimal solution for the pickup and delivery problem ==> see doc/problemDefinition.pdf
* **DONE(kinda)** Implement BFS and ASTAR, choose an algorithm and justify
  * Complexity of BFS makes it basically unable to deal with the dimensionality of the problem
* **DONE(kinda)** Implement a deliberative agent that can use the above algorithms
* **NOT DONE** Compare performance of ASTAR and BFS for various problem sizes
* **DONE(Kinda)** Run the simulation with 1,2,3 deliberative agent and report on the diffrences and joint performances of the agents

## Todo : 

* Define states, transitions and goals representations in doc **DONE**
* Implement the agent
  * **DONE(kinda)** Implement the state class
  * **DONE** The succ function for a given state class
  * **DONE** goal detection
  * **DONE** Plan generation from states
  * **DONE** some way to solve using BFS which is not atrocious
  * **DONE** implement a heuristic function that makes more sense or at least justify why the fuck it workds how it does not
  * **DONE** re-planning
  * **DONE** Run the simulation with several agents ==> it works
  * **NOT DONE** Find the max number of tasks for which one can estiblish a plan in 1 min for BFS and for ASTAR
  * **NOT DONE** Run avec 6 tasks, 1 agent, 2 agent, 3 agents
* Write a report **Kind of Done**