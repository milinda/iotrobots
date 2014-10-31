package cgl.iotrobots.collavoid.agent;

import cgl.iotrobots.collavoid.utils.Point2;

/**
 * Created by hjh on 10/30/14.
 */
public class NeighborDistComparator {
    public agent agt;
    public NeighborDistComparator(agent a){
        this.agt=a;
    }

    public java.util.Comparator getComparator() {

        return new java.util.Comparator() {

            public int compare(Object o1, Object o2) {
                if (o1 instanceof agent) {
                    return compare( (agent) o1, (agent) o2);
               }else {
                    System.err.println("comparator not found!!");
                    return 1;

                }
            }

            public int compare(agent a1, agent a2) {
                Point2<Double> pos=agt.position.getPoint2(),nb1=a1.position.getPoint2(),nb2=a2.position.getPoint2();
                Point2<Double> relativePos1=Point2.mul(pos,nb1);
                Point2<Double> relativePos2=Point2.mul(pos,nb2);
                double dist1=Point2.abs(relativePos1),dist2=Point2.abs(relativePos2);

                return (dist1 < dist2 ? -1 : (dist1 == dist2 ? 0 : 1));

            }



        };
    }
}
