package cgl.iotrobots.collavoid.commons.planners;

import java.util.Comparator;

import cgl.iotrobots.collavoid.planners.Agent;

public class Comparators {

    public static final NeighborDistComparator neighborDistComparator = new NeighborDistComparator();

    public static final ConvexHullPointsPositionComparator convexHullPointsPositionComparator = new ConvexHullPointsPositionComparator();

    public static final VectorsLexigraphicComparator vectorsLexigraphicComparator = new VectorsLexigraphicComparator();

    public static final VelocitySamplesComparator velocitySamplesComparator = new VelocitySamplesComparator();

    static class NeighborDistComparator {
        private Vector2 pos = null;

        public void setPos(Vector2 pos) {
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

    static class ConvexHullPointsPositionComparator implements Comparator<ConvexHullPoint> {
        public int compare(ConvexHullPoint chp1, ConvexHullPoint chp2) {
            if (Vector2.absSqr(chp1.getX(), chp1.getY()) <= Vector2.absSqr(chp2.getX(), chp2.getY()))
                return -1;
            else
                return 1;
        }
    }

    static class VectorsLexigraphicComparator implements Comparator<ConvexHullPoint> {

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

    static class VelocitySamplesComparator implements Comparator<VelocitySample> {

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
