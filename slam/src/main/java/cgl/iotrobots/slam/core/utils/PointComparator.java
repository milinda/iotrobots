package cgl.iotrobots.slam.core.utils;

import java.util.Comparator;

public class PointComparator implements Comparator<Point> {
    @Override
    public int compare(Point o1, Point o2) {
        if (o1.x instanceof Integer && o2.x instanceof Integer && o1.y instanceof Integer && o2.y instanceof Integer) {
            boolean equal = (Integer)o1.x < (Integer)o2.x || (o1.x.equals(o2.x) && (Integer)o1.y < (Integer)o2.y);
            return equal ? 1 : 0;
        } else if (o1.x instanceof Double && o2.x instanceof Double && o1.y instanceof Double && o2.y instanceof Double) {
            boolean equal = (Double)o1.x < (Double)o2.x || (o1.x.equals(o2.x) && (Double)o1.y < (Double)o2.y);
            return equal ? 1 : 0;
        }

        return 0;
    }
}
