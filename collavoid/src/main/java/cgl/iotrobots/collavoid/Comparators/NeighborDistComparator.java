package cgl.iotrobots.collavoid.Comparators;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import cgl.iotrobots.collavoid.utils.Vector2;

public class NeighborDistComparator {
    public Agent agt;

    public NeighborDistComparator(Agent a) {
        this.agt = a;
    }

    public java.util.Comparator getComparator() {

        return new java.util.Comparator() {

            public int compare(Object o1, Object o2) {
                if (o1 instanceof Agent && o2 instanceof Agent) {
                    return compare((Agent) o1, (Agent) o2);
                } else {
                    System.err.println("comparator not found!!");
                    return 1;

                }
            }

            public int compare(Agent a1, Agent a2) {
                Vector2 pos = agt.position.getPos(), nb1 = a1.position.getPos(), nb2 = a2.position.getPos();
                Vector2 relativePos1 = Vector2.minus(pos, nb1);
                Vector2 relativePos2 = Vector2.minus(pos, nb2);
                double dist1 = Vector2.abs(relativePos1), dist2 = Vector2.abs(relativePos2);

                return (dist1 < dist2 ? -1 : (dist1 == dist2 ? 0 : 1));

            }


        };
    }
}
