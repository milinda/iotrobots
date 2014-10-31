package cgl.iotrobots.collavoid.utils;


/**
 * Created by hjh on 10/24/14.
 */
public class Line<T> {
    public Point2<T> point;
    public Point2<T> dir;

    public Line(Point2<T> point, Point2<T> dir) {
        this.point = point;
        this.dir = dir;
    }
}

