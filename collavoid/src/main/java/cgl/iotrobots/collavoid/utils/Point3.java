package cgl.iotrobots.collavoid.utils;


/**
 * Created by hjh on 10/30/14.
 */

public class Point3<T> extends Point2 {
    public T x;
    public T y;
    public T theta;

    public Point3(T x, T y) {
        super(x, y);
    }

    public Point3(T x, T y, T theta) {
        super(x, y);
        this.theta=theta;
    }

    public Point3(Point2<T> p) {
        super(p.x, p.y);
        if (p instanceof Point3 ) {
            theta = (T) ((Point3) p).theta;
        }
    }

    public Point2<T> getPoint2(){
        return new Point2<T>(this.x,this.y);
    }

}
