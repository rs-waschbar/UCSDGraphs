/**
 * @author Ruslan Zhdanov
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
    HashMap<GeographicPoint, MapNode> mapNodes;

	public MapGraph() {
	    mapNodes = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return mapNodes.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return mapNodes.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
	    int numEdges = 0;

        for (Map.Entry<GeographicPoint, MapNode> entry : mapNodes.entrySet()) {
            MapNode v = entry.getValue();
            numEdges += v.getEdges().size();
        }
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
        if (location == null || mapNodes.containsKey(location)) return false;

        mapNodes.put(location, new MapNode(location));
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

	    if (!mapNodes.containsKey(from)
                || !mapNodes.containsKey(to)
                || length < 0
                || from == null
                || to == null
                || roadName == null
                || roadType == null) {
	        throw new IllegalArgumentException();
        }

	    MapNode currentNode = mapNodes.get(from);
        currentNode.getEdges().add(new MapEdge(currentNode, mapNodes.get(to) , roadName, roadType, length));
	}

	/** helper method for debugging  */

	public void printGraph () {
		mapNodes.forEach((k, v) -> {
			System.out.println("key: " + k + ", value: " + v);
			System.out.println("num edges: " + v.getEdges().size());
		});

	}


	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

		HashSet<MapNode> visited = new HashSet<>();
		LinkedList<MapNode> nextList = new LinkedList<>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		MapNode startNode = mapNodes.get(start);
		MapNode goalNode = mapNodes.get(goal);

		visited.add(startNode);
		nextList.add(startNode);
		MapNode currNode;

		while (!nextList.isEmpty()) {
			currNode = nextList.poll();
			nodeSearched.accept(currNode.getLocation());

			if (currNode.equals(goalNode))
				break;

			for (MapNode node : currNode.getNeighbours().keySet()) {
				if (!visited.contains(node)) {
					visited.add(node);
					parentMap.put(node, currNode);
					nextList.add(node);
				}
			}
		}

		if (!visited.contains(goalNode))
			return null;
		else
			return restorePath(startNode, goalNode, parentMap);
	}

    /** Method for restore path from start to end, after BFS
     * created to not overload search methods
     * p.s.this method will restored path from goal point to previous elements to start point
     * and return path in right from-start-to-end order.
     *
     * @param start The starting location
     * @param goal The goal location
     * @param parentMap The Map child-parent
     * @return path from start to goal (including both start and goal).
     */

	private LinkedList<GeographicPoint> restorePath (MapNode start, MapNode goal,
                                                     HashMap<MapNode, MapNode> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode curr = goal;
		path.addFirst(goal.getLocation());

        while (!curr.equals(start)) {
			curr = parentMap.get(curr);
			path.addFirst(curr.getLocation());
        }

        return path;
    }
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        HashMap<MapNode, Double> distFromStart = new HashMap<>();
        mapNodes.forEach((k, v) -> distFromStart.put(v, Double.MAX_VALUE));

        HashSet<MapNode> visited = new HashSet<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>
                (Comparator.comparingDouble(distFromStart::get));

        HashMap<MapNode, MapNode> parentMap = new HashMap<>();
        MapNode startNode = mapNodes.get(start);
        MapNode goalNode = mapNodes.get(goal);

        distFromStart.put(startNode, 0d);
        visited.add(startNode);
        queue.add(startNode);
        MapNode currNode;

        int count = 0;
        while (!queue.isEmpty()) {
            currNode = queue.poll();
            count++;
            nodeSearched.accept(currNode.getLocation());

            if (currNode.equals(goalNode))
                break;

            for (MapNode node : currNode.getNeighbours().keySet()) {
                double nDist = distFromStart.get(currNode) + currNode.getEdgeLength(node);
                if (nDist < distFromStart.get(node)) {
                    distFromStart.put(node, nDist);
                    visited.add(node);
                    parentMap.put(node, currNode);
                    queue.add(node);
                }
            }
        }
        System.out.println("Count of nodes: " + count);
        if (!visited.contains(goalNode))
            return null;
        else
            return restorePath(startNode, goalNode, parentMap);
	}


	/** Improved find the path from start to goal using Dijkstra's algorithm
	 *	takes into account the type of road when searching the result
	 * 	prefer path with better trip duration
     *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */

	public List<GeographicPoint> modDijkstra(GeographicPoint start,
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        HashMap<MapNode, Double> modDistFromStart = new HashMap<>();
        mapNodes.forEach((k, v) -> modDistFromStart.put(v, Double.MAX_VALUE));

        HashSet<MapNode> visited = new HashSet<>();
        PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>
                (Comparator.comparingDouble(modDistFromStart::get));

        HashMap<MapNode, MapNode> parentMap = new HashMap<>();
        MapNode startNode = mapNodes.get(start);
        MapNode goalNode = mapNodes.get(goal);

        modDistFromStart.put(startNode, 0d);
        visited.add(startNode);
        queue.add(startNode);
        MapNode currNode;

        int count = 0;
        while (!queue.isEmpty()) {
            currNode = queue.poll();
            count++;
            nodeSearched.accept(currNode.getLocation());

            if (currNode.equals(goalNode))
                break;

			HashMap<MapNode, MapEdge> neighbours = currNode.getNeighbours();
			for (MapNode node : neighbours.keySet()) {
				double nDist = modDistFromStart.get(currNode) +
						neighbours.get(node).lengthSpeedCoefficient();
				if (nDist < modDistFromStart.get(node)) {
                    modDistFromStart.put(node, nDist);
                    visited.add(node);
                    parentMap.put(node, currNode);
                    queue.add(node);
                }
            }
        }
        System.out.println("Count of nodes: " + count);
        if (!visited.contains(goalNode))
            return null;
        else
            return restorePath(startNode, goalNode, parentMap);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return modStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		HashMap<MapNode, Double> distFromStart = new HashMap<>();
		MapNode startNode = mapNodes.get(start);
		MapNode goalNode = mapNodes.get(goal);
		mapNodes.forEach((k, v) -> distFromStart.put(v, Double.MAX_VALUE));

		HashMap<MapNode, MapNode> parentMap = new HashMap<>();
		HashSet<MapNode> visited = new HashSet<>();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>
				(Comparator.comparingDouble(o -> distFromStart.get(o) + o.getLocation().distance(goal)));


		distFromStart.put(startNode, 0d);
		visited.add(startNode);
		queue.add(startNode);
		MapNode currNode;
		int count = 0;

		while (!queue.isEmpty()) {
			currNode = queue.poll();
			count++;
			nodeSearched.accept(currNode.getLocation());

			if (currNode.equals(goalNode))
				break;

			HashMap<MapNode, MapEdge> neighbours = currNode.getNeighbours();
			for (MapNode node : neighbours.keySet()) {
				double nDist = distFromStart.get(currNode) + currNode.getEdgeLength(node);
				if (nDist < distFromStart.get(node)) {
					distFromStart.put(node, nDist);
					visited.add(node);
					parentMap.put(node, currNode);
					queue.add(node);
				}
			}
		}
        System.out.println("Count of nodes: " + count);
		if (!visited.contains(goalNode))
			return null;
		else
			return restorePath(startNode, goalNode, parentMap);
	}

	// TODO: Week 6 Bonus extension

	/** Improved find the path from start to goal using A-Star search
	 * takes into account the type of road when searching the result
	 * prefer path with better trip duration
	 *
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from
	 *   start to goal (including both start and goal).
	 */

	public List<GeographicPoint> modStarSearch(GeographicPoint start,
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

		HashMap<MapNode, Double> modDistFromStart = new HashMap<>();
		MapNode startNode = mapNodes.get(start);
		MapNode goalNode = mapNodes.get(goal);
		mapNodes.forEach((k, v) -> modDistFromStart.put(v, Double.MAX_VALUE));

		HashSet<MapNode> visited = new HashSet<>();
		PriorityQueue<MapNode> queue = new PriorityQueue<MapNode>
				(Comparator.comparingDouble((MapNode node) ->
						modDistFromStart.get(node) + node.getLocation().distance(goal)));

		HashMap<MapNode, MapNode> parentMap = new HashMap<>();

		modDistFromStart.put(startNode, 0d);
		visited.add(startNode);
		queue.add(startNode);
		MapNode currNode;
		int count = 0;

		while (!queue.isEmpty()) {
			currNode = queue.poll();
			count++;
			nodeSearched.accept(currNode.getLocation());

			if (currNode.equals(goalNode))
				break;

			HashMap<MapNode, MapEdge> neighbours = currNode.getNeighbours();
			for (MapNode node : neighbours.keySet()) {
				double nDist = modDistFromStart.get(currNode) +
						neighbours.get(node).lengthSpeedCoefficient();
				if (nDist < modDistFromStart.get(node)) {
					modDistFromStart.put(node, nDist);
					visited.add(node);
					parentMap.put(node, currNode);
					queue.add(node);
				}
			}
		}
        System.out.println("Count of nodes: " + count);
		if (!visited.contains(goalNode))
			return null;
		else
			return restorePath(startNode, goalNode, parentMap);
	}

}