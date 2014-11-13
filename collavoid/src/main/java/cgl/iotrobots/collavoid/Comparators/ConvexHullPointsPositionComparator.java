package cgl.iotrobots.collavoid.Comparators;

import cgl.iotrobots.collavoid.utils.ConvexHullPoint;
import cgl.iotrobots.collavoid.utils.Vector2;

import java.util.Comparator;


public class ConvexHullPointsPositionComparator implements Comparator<ConvexHullPoint>{
    public int compare(ConvexHullPoint chp1,ConvexHullPoint chp2){
        if(Vector2.absSqr(chp1.getPoint())<=Vector2.absSqr(chp2.getPoint()))
            return -1;
        else
            return 1;
    }

}
