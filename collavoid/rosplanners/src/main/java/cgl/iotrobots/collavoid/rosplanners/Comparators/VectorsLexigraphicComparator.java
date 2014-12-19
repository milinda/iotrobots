package cgl.iotrobots.collavoid.rosplanners.Comparators;

import cgl.iotrobots.collavoid.rosplanners.utils.ConvexHullPoint;

import java.util.Comparator;


public class VectorsLexigraphicComparator implements Comparator<ConvexHullPoint> {

    public int compare(ConvexHullPoint c1, ConvexHullPoint c2) {
        if(null==c1||null==c2){
            return -1;
        }

        if (c1.getX() < c2.getX() || (c1.getX() == c2.getX() && c1.getY() < c2.getY())) {
            return -1;
        } else if (c1.getX() == c2.getX() && c1.getY() == c2.getY()) {
            return 0;
        }else{
            return 1;
        }

    }

}
