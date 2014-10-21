package cgl.iotrobots.slam.core.utils;

public class PointPair<T> {
    public Point<T> point1;
    public Point<T> point2;

    public PointPair(Point<T> point1, Point<T> point2) {
        this.point1 = point1;
        this.point2 = point2;
    }
}
