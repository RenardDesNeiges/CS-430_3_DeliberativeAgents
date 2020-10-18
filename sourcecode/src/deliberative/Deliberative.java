package deliberative;

import java.util.Collections;
// Importing java utils
import java.util.LinkedList;
import java.util.Stack;

import deliberative.State.Act;


/* import table */
import logist.simulation.Vehicle;
import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */

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
	/*
	 * Basic constructor
	 */
	public State(logist.topology.Topology.City city, 
			LinkedList<Task> ctask, LinkedList<Task> free_tasks, 
			State parent, double cost,Act act,int depth){
		this.city = city;
		this.ctask = ctask;
		this.free_tasks = free_tasks;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;

	}

	/*
	 * Constructor taking a TaskSet and converting it to a LinkedList<Task>
	 */
	public State(logist.topology.Topology.City city, 
			LinkedList<Task> ctask,
			TaskSet free_tasks, State parent, double cost,Act act,int depth) {
		this.city = city;
		this.ctask = ctask;
		this.parent = parent;
		this.cost = cost;
		this.act = act;
		this.depth = depth;
			
		//converting TaskSet to LinkedList<Task>
		LinkedList<Task> lFreeTasks = new LinkedList<Task>();
		for(Task tsk : free_tasks){
			lFreeTasks.add(tsk);
		}
		this.free_tasks = lFreeTasks;
	}

	/*
	 * Compute the weight carried by the agent in this state
	 */
	public long cweight() {
		long	sum = 0;
		for(Task task :this.ctask){
			sum = sum + task.weight;
		}
		return sum;
	}

	public double heuristic(){
		// C'est uNE hEUriSTIqUE (j'ai mis de constantes au bol, de façon heuristique tsais)
		return 24000*this.free_tasks.size()+12000*this.ctask.size();
	}

	/*
	 * Generates all possible states the agent might transition to from a given start
	 * state (the successors on the graph)
	 */
	public Stack<State> succ(Topology topology,int capacity){
		Stack<State> succ = new Stack<State>();
		
		//generating movement actions
		for (City stop : topology) {
			if (this.city.hasNeighbor(stop)) {
				State nState = new State(stop, this.ctask, this.free_tasks, this,
						this.cost + this.city.distanceTo(stop) * capacity,Act.MOVE,this.depth+1);
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
				
				State nState = new State(this.city, nCTask, nFreeTasks, this, this.cost,Act.PICKUP,this.depth+1);
				succ.push(nState);
			}
		}

		//generating delivery actions
		for (Task task : this.ctask) {
			if (task.deliveryCity == this.city) {

				LinkedList nCTask = new LinkedList();
				nCTask = (LinkedList) this.ctask.clone();
				nCTask.remove(task);

				State nState = new State(this.city, nCTask, this.free_tasks, this, this.cost,Act.DELIVER,this.depth+1);
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
	@Override
	public String toString() {
		return this.city + "; cost: " + this.cost + "; act: "+ this.act + "; freeTasks : " + this.free_tasks.size()+"; depth: "+this.depth;
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

	/* the planning class */
	Algorithm algorithm;
	
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;


		
		
		// initialize the planner
		this.capacity = agent.vehicles().get(0).capacity();
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");

		
		// Throws IllegalArgumentException if algorithm is unknown
		algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
		
		// ...
	}
	
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan;

		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			// ...
			plan = astarPlanGenerator(vehicle, tasks);
			break;
		case BFS:
			// ...
			plan = astarPlanGenerator(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}

	private State bfsAlgorithm(Vehicle vehicle, TaskSet tasks){
		LinkedList<State> Q = new LinkedList<State>();
		Q.add(new State(vehicle.getCurrentCity(), new LinkedList<Task>(), tasks, null, 0.0,Act.START,0)); //initializing the stack with the root node
		while (Q.size()>0){
		Collections.sort(Q);
		State node = Q.removeFirst();
		if(node.isGoal()){
			return node;
		}
		//Pushing the new states into the stack
		Stack<State> newStates = node.succ(this.topology, this.capacity);

		for (State st : newStates) {
			Q.add(st);
		}
		System.out.println("depth : " + node.depth + "; free-tasks: " + node.free_tasks.size()+ "; ctasks : " + node.ctask.size() );

	}
		return null;
	}

	private State astarAlgorithm(Vehicle vehicle, TaskSet tasks) {
		LinkedList<State> Q = new LinkedList<State>();
		int dpth = 0;

		Q.add(new State(vehicle.getCurrentCity(), new LinkedList<Task>(), tasks, null, 0.0, Act.START, 0)); // initializing the stack with the root node
		while (Q.size() > 0) {
			Collections.sort(Q);
			State node = Q.removeFirst();
			if (node.isGoal()) {
				return node;
			}
			// Pushing the new states into the stack
			Stack<State> newStates = node.succ(this.topology, this.capacity);

			for (State st : newStates) {
				Q.add(st);
			}
			double val = node.cost+node.heuristic();
			/*System.out.println("depth : " + node.depth + "; free-tasks: " + node.free_tasks.size() + "; ctasks : "
					+ node.ctask.size() + "; h : " + val );*/
			if (node.depth > dpth) {
				dpth = node.depth;
			}

		}
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
			//System.out.println(i + ", s: " + path.get(i));
			//System.out.println(i + ", f: " + path.get(i).free_tasks);
			//System.out.println(i + ", c: " + path.get(i).ctask);
			if(path.get(i).act == Act.MOVE){
				plan.appendMove(path.get(i).city);
			}
			else if(path.get(i).act == Act.PICKUP){
				//System.out.print("Picked up ");
				for(Task tsk : path.get(i).ctask){
					if(tsk.pickupCity == path.get(i).city){
						plan.appendPickup(tsk);
						//System.out.println(tsk);
						break;
					}
				}
			}
			else if(path.get(i).act == Act.DELIVER){
				//System.out.println(path.get(i+1).ctask);
				//System.out.print("Delivered ");
				for (Task tsk : path.get(i+1).ctask) {
					//System.out.println(tsk+ " // " + path.get(i).city);
					if (tsk.deliveryCity == path.get(i).city) {
						//System.out.println(tsk);
						plan.appendDelivery(tsk);
						break;
					}
				}
			}
			//System.out.println("");
		}
		return plan;
	}
	
	private Plan astarPlanGenerator(Vehicle vehicle, TaskSet tasks) {
		Plan plan = backtrack(astarAlgorithm(vehicle, tasks));
		//System.out.println(plan);
		///// TESTING ENV
		///////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////
		/*for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}*/
		return plan;
	}

	private Plan bfsPlanGenerator(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		///// TESTING ENV
		///////////////////////////////////////////////////////////////////////////////////////////
		backtrack(bfsAlgorithm(vehicle, tasks));

		///////////////////////////////////////////////////////////////////////////////////////////
		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}
	
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	@Override
	public void planCancelled(TaskSet carriedTasks) {
		
		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
		}
	}
}
