package cgl.iotrobots.collavoid.planners;

import cgl.iotrobots.collavoid.commons.planners.ConvexHullPoint;
import cgl.iotrobots.collavoid.commons.planners.Vector2;

import java.util.Comparator;

// separated from commons to avoid cycle references
public class NeighborDistComparator implements Comparator<Agent> {


//    public static class ConvexHullPointsPositionComparator implements Comparator<Agent> {
//        public int compare(ConvexHullPoint chp1, ConvexHullPoint chp2) {
//            if (Vector2.absSqr(chp1.getX(), chp1.getY()) <= Vector2.absSqr(chp2.getX(), chp2.getY()))
//                return -1;
//            else
//                return 1;
//        }
//    }


    private Vector2 pos = null;

    public NeighborDistComparator(double x, double y) {
        this.pos = new Vector2(x, y);
    }

            public int compare(Agent a1, Agent a2) {
                Vector2 nb1 = a1.getPosition().getPos(), nb2 = a2.getPosition().getPos();
                Vector2 relativePos1 = Vector2.minus(pos, nb1);
                Vector2 relativePos2 = Vector2.minus(pos, nb2);
                double dist1 = Vector2.abs(relativePos1), dist2 = Vector2.abs(relativePos2);

                return (dist1 < dist2 ? -1 : (dist1 == dist2 ? 0 : 1));

            }


    }

