package cgl.iotrobots.slam.core.utils;

public class Point<T> {
    public T x;
    public T y;

    public Point(Point<T> p) {
        this.x = p.x;
        this.y = p.y;
    }

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

    public static Point<Integer> max(Point<Integer> p1, Point<Integer> p2){
        Point<Integer> p = p1;
        p.x = p.x > p2.x ? p.x : p2.x;
        p.y = p.y > p2.y ? p.y : p2.y;
        return p;
    }

    public static Point<Integer> min(Point<Integer> p1, Point<Integer> p2){
        Point<Integer> p = p1;
        p.x = p.x < p2.x ? p.x : p2.x;
        p.y = p.y < p2.y ? p.y : p2.y;
        return p;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Point point = (Point) o;

        if (x != null ? !x.equals(point.x) : point.x != null) return false;
        if (y != null ? !y.equals(point.y) : point.y != null) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = x != null ? x.hashCode() : 0;
        result = 31 * result + (y != null ? y.hashCode() : 0);
        return result;
    }
}
