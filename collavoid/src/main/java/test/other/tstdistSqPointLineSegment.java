package test.other;

import cgl.iotrobots.collavoid.utils.Vector2;
import cgl.iotrobots.collavoid.utils.utils;

/**
 * Created by hjh on 12/5/14.
 */
public class tstdistSqPointLineSegment {
    public static void main(String[] args){
        Vector2 a=new Vector2(4.5,-1);
        Vector2 b=new Vector2(4.5,1);
        Vector2 c=new Vector2(6,0);
        System.out.println(utils.distSqPointLineSegment(a,b,c));

    }
}
