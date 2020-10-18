package deliberative;

// Importing java utils
import java.util.LinkedList;
import java.util.Random;

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

class State{
	public logist.topology.Topology.City city;
	public logist.task.TaskSet ctask;
	public logist.task.TaskSet free_tasks;
	public State parent;
	public double cost;

	public State(logist.topology.Topology.City city, logist.task.TaskSet ctask, logist.task.TaskSet free_tasks, 
			State parent, double cost){
		this.city = city;
		this.ctask = ctask;
		this.free_tasks = free_tasks;
		this.parent = parent;
		this.cost = cost;
	}

	public int cweight() {
		return this.ctask.weightSum();
	}

	public LinkedList<State> succ(Topology topology,int capacity){
		LinkedList<State> succ = new LinkedList<State>();
		
		//generating movement actions
		for (City stop : topology) {
			if(this.city.hasNeighbor(stop)){
				State nState = new State(stop, this.ctask, this.free_tasks, this, this.cost+this.city.distanceTo(stop)* capacity); // Implement cost function
				succ.add(nState);
			}
		}
		//generating pickup actions

		return succ;
	}

	@Override
	public String toString() {
		return this.city + ", cost:" + this.cost;
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
		int capacity = agent.vehicles().get(0).capacity();
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
			plan = BFS(vehicle, tasks);
			break;
		case BFS:
			// ...
			plan = BFS(vehicle, tasks);
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}

	private Plan BFS(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		///// TESTING ENV
		///////////////////////////////////////////////////////////////////////////////////////////
		State testState = new State(vehicle.getCurrentCity(), null, null, null, 0);
		System.out.println(testState.succ(this.topology, agent.vehicles().get(0).capacity()));
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
