package deliberative;

import java.util.LinkedList;
import java.util.Stack;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Stack;
import java.lang.Math;

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
/* import table */
import logist.simulation.Vehicle;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;
import logist.task.Task;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

class State implements Comparable<State>{
	enum Act {
		MOVE, PICKUP, DELIVER, START
	}
	public logist.topology.Topology.City city;
	public LinkedList<Task> ctask;
	public LinkedList<Task> free_tasks;
	public State parent;
	public double cost;
	public Act act;
	public int depth;
	public Task tsk;
	private Boolean heuristic;
	/*
	 * Basic constructor
	 */
	public State(logist.topology.Topology.City city, 
			LinkedList<Task> ctask, LinkedList<Task> free_tasks, 
			State parent, double cost,Act act,int depth,Boolean heuristic){
		this.city = city;
		this.ctask = ctask;
		this.free_tasks = free_tasks;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;
		this.heuristic = heuristic;
		this.tsk = null;

	}
	
	public State(logist.topology.Topology.City city, LinkedList<Task> ctask, LinkedList<Task> free_tasks, State parent,
			double cost, Act act, int depth, Boolean heuristic,Task tsk) {
		this.city = city;
		this.ctask = ctask;
		this.free_tasks = free_tasks;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;
		this.heuristic = heuristic;
		this.tsk = tsk;

	}

	/*
	 * Constructor taking a TaskSet and converting it to a LinkedList<Task>
	 */
	public State(logist.topology.Topology.City city, 
			LinkedList<Task> ctask,
			TaskSet free_tasks, State parent, double cost,Act act,int depth,Boolean heuristic) {
		this.city = city;
		this.ctask = ctask;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;
		this.heuristic = heuristic;
		this.tsk = null;
			
		//converting TaskSet to LinkedList<Task>
		LinkedList<Task> lFreeTasks = new LinkedList<Task>();
		for(Task tsk : free_tasks){
			lFreeTasks.add(tsk);
		}
		this.free_tasks = lFreeTasks;
	}

	public State(logist.topology.Topology.City city, TaskSet ctask, TaskSet free_tasks, State parent,
			double cost, Act act, int depth,Boolean heuristic) {
		this.city = city;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;
		this.heuristic = heuristic;
		this.tsk = null;

		// converting TaskSet to LinkedList<Task>
		LinkedList<Task> lFreeTasks = new LinkedList<Task>();
		for (Task tsk : free_tasks) {
			lFreeTasks.add(tsk);
		}
		LinkedList<Task> lCTask = new LinkedList<Task>();
		if(ctask != null){
			for (Task tsk : ctask) {
				lCTask.add(tsk);
			}
		}
		this.ctask = lCTask;
		this.free_tasks = lFreeTasks;
	}

	/*
	 * Compute the weight carried by the agent in this state
	 */
	public long cweight() {
		long sum = 0;
		for(Task task :this.ctask){
			sum = sum + task.weight;
		}
		return sum;
	}

	public double heuristic(){
		if(this.heuristic) {
			if(this.free_tasks.size() != 0) {
				double max_free = Double.NEGATIVE_INFINITY;
				for(Task task: this.free_tasks) {
					double x = this.city.distanceTo(task.pickupCity) + task.pickupCity.distanceTo(task.deliveryCity);
					if(max_free < x)
						max_free = x;
				}
				return max_free;
			}
			
			else {
				double max_carry = Double.NEGATIVE_INFINITY;
				for(Task task: this.ctask) { 
					double x = this.city.distanceTo(task.deliveryCity);
					if(max_carry < x)
						max_carry = x;
				}
				return max_carry;
			}
		}
		else
			return 0;
		
	}

	/*
	 * Generates all possible states the agent might transition to from a given start
	 * state (the successors on the graph)
	 */
	public Stack<State> succ(Topology topology,int capacity){
		Stack<State> succ = new Stack<State>();
		
		//generating delivery actions
		for (Task task : this.ctask) {
			if (task.deliveryCity == this.city) {

				LinkedList nCTask = new LinkedList();
				nCTask = (LinkedList) this.ctask.clone();
				nCTask.remove(task);

				State nState = new State(this.city, nCTask, this.free_tasks, this, this.cost,Act.DELIVER,this.depth+1,this.heuristic,task);
				succ.push(nState);
			}
		}
		
		if(succ.size() != 0) {
			Stack<State> res = new Stack<State>();
			res.push(succ.pop());
			return res;
		}
		
		//generating movement actions
		for (City stop : topology) {
			if (this.city.hasNeighbor(stop)) {
				State nState = new State(stop, this.ctask, this.free_tasks, this,
						this.cost + this.city.distanceTo(stop),Act.MOVE,this.depth+1,this.heuristic);
				succ.push(nState);
			}
		}
		//generating pickup actions
		for (Task task : this.free_tasks) {
			//checking wether or not the task is possible to pickup in the state
			if(task.pickupCity == this.city && task.weight<(capacity - this.cweight())){
				///// WARNING APPARENTLY THIS IS UNSAFE TYPECASTING 
				LinkedList nCTask = new LinkedList();
				nCTask = (LinkedList) this.ctask.clone();
				nCTask.add(task);
				LinkedList nFreeTasks = new LinkedList();
				nFreeTasks = (LinkedList) this.free_tasks.clone();
				nFreeTasks.remove(task);
				
				State nState = new State(this.city, nCTask, nFreeTasks, this, this.cost,Act.PICKUP,this.depth+1,this.heuristic);
				succ.push(nState);
			}
		}

		

		//We return a linked list of possible successor states
		return succ;
	}

	/*
	 * Returns true if state is a goal state
	 */
	public Boolean isGoal(){
		if(this.free_tasks.size() == 0 && this.ctask.size() == 0){
			return true;
		}
		return false;
	}

	/*
	 * When print is called on a State object, this is what is gets printed
	 */
	
	public Boolean rajoute(LinkedList<State> C) {
		if (this.act == Act.START) {
			C.add(this);
			return true;
		}
		
		for (State st: C) {
			if (st.city == this.city && st.free_tasks == this.free_tasks && st.ctask == this.ctask && st.act == this.act) {
				if(this.cost <= st.cost) {
					C.add(this);
					return true;
				}
				return false;
			}
		}
		
		C.add(this);
		return true;
	}
		
	@Override
	public String toString() {
		return this.city + "; cost: " + this.cost + "; act: "+ this.act + "; freeTasks : " + this.free_tasks.size()+ "; carried tasks: " + this.ctask.size() + "; depth: "+this.depth;
						
	}
	@Override
	public int compareTo(State cmp){
		Double cst = this.cost+this.heuristic();
		return cst.compareTo(cmp.cost+cmp.heuristic());
	}
}	

public class Deliberative implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	
	/* the properties of the agent */
	Agent agent;
	int capacity;
	TaskSet planInitTasks;

	/* the planning class */
	Algorithm algorithm;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;

		this.planInitTasks = null;
		
		System.out.println("Running Setup");
		
		
		// initialize the planner
		this.capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		System.out.println("Setup Done");
	}
	
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;
		
		System.out.println("Generating Plan");

		// Compute the plan with the selected algorithm.
		switch (algorithm) {

		case ASTAR:
			plan = astarPlanGenerator(vehicle, tasks);
			break;
			
		case BFS:
			plan = bfsPlanGenerator(vehicle, tasks);
			break;
			
		default:
			throw new AssertionError("Algorithm not defined.");
			
		}
		
		System.out.println("Plan Generated");
		return plan;
	}

	private State astarAlgorithm(Vehicle vehicle, TaskSet tasks,Boolean heuristic) {
		LinkedList<State> Q = new LinkedList<State>();
		LinkedList<State> C = new LinkedList<State>();
		int dpth = 0;
		int qsize = 0;
		
		if(heuristic)
			System.out.println("Running A*");
		else
			System.out.println("Running BFS");

		Q.add(new State(vehicle.getCurrentCity(), this.planInitTasks, tasks, null, 0.0, State.Act.START, 0, heuristic)); // initializing the stack with the root node
		while (Q.size() > 0) {
			qsize = Q.size();
			if(qsize%500 == 0)
				System.out.println("Size of Q: " + Q.size());
			
			State node = Q.removeFirst();
			
			if(node.isGoal())
				return node;
			if(node.act == State.Act.START) {
				for (State st: node.succ(topology, capacity))
					Q.add(st);
				Collections.sort(Q);
			}
			
			if(node.rajoute(C)) {
				for (State st: node.succ(topology, this.capacity))
					Q.add(st);
				Collections.sort(Q);
			}
			
			if (node.isGoal()) {
				return node;
			}
			if (node.depth > dpth) {
				dpth = node.depth;
			}
		}
		
		System.out.println("A* terminated");
		
		return null;
	}

	private Plan backtrack(State endstate){
		LinkedList<State> path = new LinkedList<State>();
		State current = endstate;
		while(current != null){
			path.add(current);
			current = current.parent;
		}
		Plan plan = new Plan(path.get(path.size()-1).city);
		for (int i = path.size(); i-- > 0;) {
			if(path.get(i).act == State.Act.MOVE){
				plan.appendMove(path.get(i).city);
			}
			else if(path.get(i).act == State.Act.PICKUP){
				plan.appendPickup(path.get(i).ctask.getLast());
			}
			else if(path.get(i).act == State.Act.DELIVER){
				plan.appendDelivery(path.get(i).tsk);
			}
		}
		return plan;
	}
	
	private Plan astarPlanGenerator(Vehicle vehicle, TaskSet tasks) {
		Plan plan = backtrack(astarAlgorithm(vehicle, tasks,true));
		System.out.println(vehicle.name()+"'s plan");
		System.out.println(plan);
		return plan;
	}

	private Plan bfsPlanGenerator(Vehicle vehicle, TaskSet tasks) {
		Plan plan = backtrack(astarAlgorithm(vehicle, tasks, false));
		System.out.println(vehicle.name()+"'s plan");
		System.out.println(plan);
		return plan;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		System.out.println(agent.vehicles().get(0).name()+ "'s plan not feasible anymore, recomputing....");
		if (!carriedTasks.isEmpty()) {
			this.planInitTasks = carriedTasks;
		}
	}
}


