package cgl.iotrobots.collavoid.ClearPath;

import cgl.iotrobots.collavoid.utils.ConvexHullPoint;
import cgl.iotrobots.collavoid.utils.Vector2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by hjh on 11/1/14.
 */
public class CP {

    public static List<Vector2> minkowskiSum(final List<Vector2> polygon1, final List<Vector2> polygon2){
        List<Vector2> result=new ArrayList<Vector2>();
        List< ConvexHullPoint > convex_hull=new ArrayList<ConvexHullPoint>();

        for (int i = 0; i <  polygon1.size(); i++) {
            for (int j = 0; j <  polygon2.size(); j++) {
                ConvexHullPoint p=new ConvexHullPoint();
                p.setPoint(Vector2.plus(polygon1.get(i),polygon2.get(j)));
                convex_hull.add(p);
            }

        }
        convex_hull = convexHull(convex_hull,false);
        for (int i = 0; i< convex_hull.size(); i++) {
            result.add(convex_hull.get(i).getPoint());
        }
        return result;

    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    //Wikipedia Monotone chain...
    private static List<ConvexHullPoint > convexHull(List<ConvexHullPoint > P, boolean sorted)
    {
        int n = P.size(), k = 0;
        List<ConvexHullPoint> result=new ArrayList<ConvexHullPoint>(2*n);

        // Sort points lexicographically
        if (!sorted)
        Collections.sort(P,new VectorsLexigraphicComparator());


        //    ROS_WARN("points length %d", (int)P.size());

        // Build lower hull
        for (int i = 0; i < n; i++) {
            while (k >= 2 && cross(result.get(k-2), result.get(k-1), P.get(i)) <= 0) k--;
            result.add(k++,P.get(i));
        }

        // Build upper hull
        for (int i = n-2, t = k+1; i >= 0; i--) {
            while (k >= t && cross(result.get(k-2), result.get(k-1), P.get(i)) <= 0) k--;
            result.add(k++,P.get(i));
        }
        //result.resize(k);

        return result;
    }

    static double cross(final ConvexHullPoint o, final ConvexHullPoint A, final ConvexHullPoint B){
        return (A.getPoint().getX()- o.getPoint().getX()) * (B.getPoint().getY() - o.getPoint().getY())
                -(A.getPoint().getY() - o.getPoint().getY()) * (B.getPoint().getX() - o.getPoint().getX());
    }


}
