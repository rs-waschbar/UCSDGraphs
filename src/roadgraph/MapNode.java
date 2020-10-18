package roadgraph;

import geography.GeographicPoint;

import java.util.*;

/**
 * A class which contains graphNode (intersections)
 * and list of its adjoining output edges
 */

class MapNode {
    private final GeographicPoint location;
    private HashSet<MapEdge> edges;

    public MapNode(GeographicPoint location, HashSet<MapEdge> edges) {
        this.location = location;
        this.edges = edges;
    }

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new HashSet<>();
    }

    public HashSet<MapEdge> getEdges() {
        return edges;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public double getEdgeLength (MapNode goal) {
        double tempDist = 0;
        double minDist = Double.MAX_VALUE;

        for (MapEdge edge : edges) {
            if (edge.getStart() == this && edge.getEnd() == goal) {
                tempDist = edge.getLength();
                if (tempDist < minDist) {
                    minDist = tempDist;
                }
            }
        }
        return minDist;
    }

    /**
     * @return list of neighbours GeographicPoints for current MapNode
     */

    public HashMap<MapNode, MapEdge> getNeighbours() {
        HashMap<MapNode, MapEdge> neighbours = new HashMap<>();

        for (MapEdge edge : edges) {
            MapNode neighbour = edge.getEnd();
            neighbours.put(neighbour, edge);
        }
        return neighbours;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof MapNode))
            return false;
        MapNode node = (MapNode) obj;

        return this.location.equals(node.location);
    }

    @Override
    public String toString() {
        return "MapNode: " + this.getLocation() + " ";
    }
}
