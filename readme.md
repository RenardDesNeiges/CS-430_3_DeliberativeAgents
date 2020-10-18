# Reactive Agents, CS-430 Assignement 3

![screenshot](screenshot.png)
Implementation of a planning deliberative agent with graph-search algorithms.

## Tasks from the problem definition :
* **DONE** Choose an adequate reprensentation of **states**, **transitions** and **goals** (final states), such that BFS or ASTAR will be able to find an optimal solution for the pickup and delivery problem ==> see doc/problemDefinition.pdf
* **NOT DONE** Implement BFS and ASTAR, choose an algorithm and justify
* **NOT DONE** Implement a deliberative agent that can use the above algorithms
* **NOT DONE** Compare performance of ASTAR and BFS for various problem sizes
* **NOT DONE** Run the simulation with 1,2,3 deliberative agent and report on the diffrences and joint performances of the agents

## Todo : 

* Define states, transitions and goals representations in doc **DONE**
* Implement the agent
  * **DONE(kinda)** Implement the state class
  * **NOT DONE** The succ function for a given state class
  * **NOT DONE** goal detection
  * **NOT DONE** Plan generation from states
* Write a report