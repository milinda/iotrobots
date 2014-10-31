package cgl.iotrobots.slam.core.utils;

public class Point<T> {
    public T x;
    public T y;

    public Point(T x, T y) {
        this.x = x;
        this.y = y;
    }

    public static Point<Double> minus(Point<Double> p1, Point<Double> p2) {
        double x = p1.x - p2.x;
        double y = p1.y - p2.y;
        return new Point<Double>(x, y);
    }

    public static Point<Double> plus(Point<Double> p1, Point<Double> p2) {
        double x = p1.x + p2.x;
        double y = p1.y + p2.y;
        return new OrientedPoint<Double>(x, y);
    }

    public static Point<Double> mul(Point<Double> p1, Point<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return new OrientedPoint<Double>(x, y);
    }

    public static double mulD(Point<Double> p1, Point<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return x + y;
    }

    public static double mulN(OrientedPoint<Double> p1, OrientedPoint<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return x + y;
    }
}
