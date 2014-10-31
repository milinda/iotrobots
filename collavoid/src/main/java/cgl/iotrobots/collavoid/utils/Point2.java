package cgl.iotrobots.collavoid.utils;

import geometry_msgs.Point32;
//revised from slam.core.utlils
public class Point2<T>{
    public T x;
    public T y;

    public Point2(T x, T y) {
        this.x = x;
        this.y = y;
    }
    public Point2(Point2<T> p) {
        this.x = p.x;
        this.y = p.y;
    }

    public static Point2<Double> minus(Point2<Double> p1, Point2<Double> p2) {
        double x = p1.x - p2.x;
        double y = p1.y - p2.y;
        return new Point2<Double>(x, y);
    }

    public static Point2<Double> plus(Point2<Double> p1, Point2<Double> p2) {
        double x = p1.x + p2.x;
        double y = p1.y + p2.y;
        return new Point2<Double>(x, y);
    }

    public static Point2<Double> mul(Point2<Double> p1, Point2<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return new Point2<Double>(x, y);
    }

    public static double mulN(Point2<Double> p1, Point2<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return x + y;
    }

    public static double abs(Point2<Double> p1) {
        double x = p1.x * p1.x;
        double y = p1.y * p1.y;
        return Math.sqrt(x + y);
    }


    //convert pointcloud point32 datatype to Point2
    public static Point2<Double> Point32ToPoint2(Point32 p){
        Double x = new Double(p.getX());
        Double y = new Double(p.getY());
        return new Point2<Double>(x, y);
    }

    public static Point2<Double> rotateVectorByAngle(Point2<Double> p, double ang){
        double cos_a, sin_a,x,y;
        cos_a = Math.cos(ang);
        sin_a = Math.sin(ang);
        x=cos_a * p.x - sin_a * p.y;
        y=cos_a * p.y + sin_a * p.x;
        return new Point2<Double>(x,y);
    }
}
