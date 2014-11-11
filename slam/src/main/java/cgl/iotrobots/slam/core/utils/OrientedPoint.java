package cgl.iotrobots.slam.core.utils;

public class OrientedPoint<T> extends Point<T> {
    public T theta;

    public OrientedPoint(T x, T y) {
        super(x, y);
    }

    public OrientedPoint(T x, T y, T theta) {
        super(x, y);
        this.theta = theta;
    }

    public OrientedPoint(Point<T> p) {
        super(p.x, p.y);
        if (p instanceof OrientedPoint) {
            theta = (T) ((OrientedPoint) p).theta;
        }
    }

    public static OrientedPoint<Double> minus(OrientedPoint<Double> p1, OrientedPoint<Double> p2) {
        double x = p1.x - p2.x;
        double y = p1.y - p2.y;
        double theta = p1.theta - p2.theta;
        return new OrientedPoint<Double>(x, y, theta);
    }

    public static OrientedPoint<Double> plus(OrientedPoint<Double> p1, OrientedPoint<Double> p2) {
        double x = p1.x + p2.x;
        double y = p1.y + p2.y;
        double theta = p1.theta + p2.theta;
        return new OrientedPoint<Double>(x, y, theta);
    }

    public static OrientedPoint<Double> mul(OrientedPoint<Double> p1, OrientedPoint<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
//        double theta = p1.theta + p2.theta;
        return new OrientedPoint<Double>(x, y, p1.theta);
    }

    public static double mulN(OrientedPoint<Double> p1, OrientedPoint<Double> p2) {
        double x = p1.x * p2.x;
        double y = p1.y * p2.y;
        return x + y;
    }

    public static OrientedPoint<Double> mulN(OrientedPoint<Double> p1, double p2) {
        double x = p1.x * p2;
        double y = p1.y * p2;
        return new OrientedPoint<Double>(x, y, p1.theta * p2);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;

        OrientedPoint that = (OrientedPoint) o;

        if (theta != null ? !theta.equals(that.theta) : that.theta != null) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result = super.hashCode();
        result = 31 * result + (theta != null ? theta.hashCode() : 0);
        return result;
    }
}
