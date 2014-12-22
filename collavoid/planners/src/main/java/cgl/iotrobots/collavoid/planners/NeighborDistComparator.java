package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.Vector2;

// separated from commons to avoid cycle references
public class NeighborDistComparator {
    private Vector2 pos = null;

    public NeighborDistComparator(Vector2 pos) {
        this.pos = pos;
    }

    public java.util.Comparator getComparator() {

        return new java.util.Comparator() {

            public int compare(Object o1, Object o2) {
                if (o1 instanceof Agent && o2 instanceof Agent && pos != null) {
                    return compare(o1, o2);
                } else {
                    System.err.println("comparator not found!!");
                    return 1;

                }
            }

            public int compare(Agent a1, Agent a2) {
                Vector2 nb1 = a1.getPosition().getPos(), nb2 = a2.getPosition().getPos();
                Vector2 relativePos1 = Vector2.minus(pos, nb1);
                Vector2 relativePos2 = Vector2.minus(pos, nb2);
                double dist1 = Vector2.abs(relativePos1), dist2 = Vector2.abs(relativePos2);

                return (dist1 < dist2 ? -1 : (dist1 == dist2 ? 0 : 1));

            }


        };
    }
}
