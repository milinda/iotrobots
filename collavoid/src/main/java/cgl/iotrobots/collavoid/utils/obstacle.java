package cgl.iotrobots.collavoid.utils;

import org.ros.message.Time;

/**
 * Created by hjh on 10/24/14.
 */
public class obstacle {
    public Point2<Double> point1;
    public Point2<Double> point2;
    public Time lastSeen;

    public obstacle(Point2<Double> p1,Point2<Double> p2){
        point1=new Point2<Double>(p1.x,p1.y);
        point2=new Point2<Double>(p2.x,p2.y);
    }
}
