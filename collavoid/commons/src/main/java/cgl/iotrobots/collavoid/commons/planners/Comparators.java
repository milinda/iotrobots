package cgl.iotrobots.collavoid.commons.planners;

import java.util.Comparator;

public class Comparators {

    public static class ConvexHullPointsPositionComparator implements Comparator<ConvexHullPoint> {
        public int compare(ConvexHullPoint chp1, ConvexHullPoint chp2) {
            if (Vector2.absSqr(chp1.getX(), chp1.getY()) <= Vector2.absSqr(chp2.getX(), chp2.getY()))
                return -1;
            else
                return 1;
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
