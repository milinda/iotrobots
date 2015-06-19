package cgl.iotrobots.collavoid.commons.planners;

import java.io.Serializable;
import java.util.Comparator;

public class Comparators implements Serializable {

    public static class NeighborDistComparator implements Comparator<Neighbor> {

        private Vector2 pos = null;

        public NeighborDistComparator(Vector2 pose) {
            this.pos = pose;
        }

        public int compare(Neighbor a1, Neighbor a2) {
            Vector2 nb1 = a1.getPosition().getPos(), nb2 = a2.getPosition().getPos();
            Vector2 relativePos1 = Vector2.minus(pos, nb1);
            Vector2 relativePos2 = Vector2.minus(pos, nb2);
            double dist1 = Vector2.abs(relativePos1), dist2 = Vector2.abs(relativePos2);
            return (dist1 < dist2 ? -1 : (dist1 == dist2 ? 0 : 1));
        }
    }

    public static class VectorsLexigraphicComparator implements Comparator<ConvexHullPoint> {

        public int compare(ConvexHullPoint c1, ConvexHullPoint c2) {
            if (null == c1 || null == c2) {
                return -1;
            }

            if (c1.getX() < c2.getX() || (c1.getX() == c2.getX() && c1.getY() < c2.getY())) {
                return -1;
            } else if (c1.getX() == c2.getX() && c1.getY() == c2.getY()) {
                return 0;
            } else {
                return 1;
            }
        }
    }

    public static class VelocitySamplesComparator implements Comparator<VelocitySample> {

        public int compare(VelocitySample vs1, VelocitySample vs2) {
            if (vs1.getDistToPrefVel() < vs2.getDistToPrefVel()) {
                return -1;
            } else if (vs1.getDistToPrefVel() > vs2.getDistToPrefVel()) {
                return 1;
            } else
                return 0;
        }
    }
}
