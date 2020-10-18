package roadgraph;

import geography.GeographicPoint;

/**
 * A class which represents paths between Intersections/MapNodes
 */

class MapEdge {
    private final MapNode start;
    private final MapNode end;
    private final String streetName;
    private final String streetType;
    private final double length;

    public MapEdge(MapNode startLoc, MapNode end,
                   String streetName, String streetType, double length) {
        this.start = startLoc;
        this.end = end;
        this.streetName = streetName;
        this.streetType = streetType;
        this.length = length;
    }

    public double lengthSpeedCoefficient() {
        if (streetType.equals("motorway") || streetType.equals("motorway_link"))
            return length * 0.6;
        else if (streetType.equals("primary") || streetType.equals("primary_link"))
            return length * 0.8;
        else if (streetType.equals("secondary") || streetType.equals("secondary_link"))
            return length * 1;
        else if (streetType.equals("tertiary") || streetType.equals("tertiary_link"))
            return length * 1.2;
        else if (streetType.equals("residential") || streetType.equals("living_street"))
            return length * 1.4;
        else
            return length * 1.5;
    }

    public MapNode getOtherEnd(MapNode node) {
        if (node.equals(this.start))
            return end;
        else if (node.equals(this.end))
            return start;
        else
            throw new IllegalArgumentException("Input Node is not in the Edge");
    }

    public MapNode getStart() {
        return start;
    }

    public MapNode getEnd() {
        return end;
    }

    public double getLength() {
        return length;
    }

    public GeographicPoint getStartLoc() {
        return start.getLocation();
    }

    public GeographicPoint getEndLoc() {
        return end.getLocation();
    }

    public String getStreetName() {
        return streetName;
    }

    public String getStreetType() {
        return streetType;
    }
}
