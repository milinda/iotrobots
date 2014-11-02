package cgl.iotrobots.collavoid.ClearPath;

import cgl.iotrobots.collavoid.utils.ConvexHullPoint;

import java.util.Comparator;

/**
 * Created by hjh on 11/1/14.
 */
public class VectorsLexigraphicComparator implements Comparator<ConvexHullPoint> {

    public int compare(ConvexHullPoint c1, ConvexHullPoint c2) {
        if(null==c1||null==c2){
            return -1;
        }

        if(c1.getPoint().getX()<c2.getPoint().getX()||(c1.getPoint().getX()==c2.getPoint().getX()&&c1.getPoint().getY()<c2.getPoint().getY())){
            return -1;
        }else if(c1.getPoint().getX()==c2.getPoint().getX()&&c1.getPoint().getY()==c2.getPoint().getY()){
            return 0;
        }else{
            return 1;
        }

    }

}
