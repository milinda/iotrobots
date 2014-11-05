package cgl.iotrobots.collavoid.utils;

public class Angles {

    public static double normalize_angle_positive(double angle) {
        return ((angle % 2.0 * Math.PI) + 2.0 * Math.PI) % 2.0 * Math.PI;//why mod twice??
    }

    public static double normalize_angle(double angle) {
        double a = normalize_angle_positive(angle);
        if (a > Math.PI)
            a -= 2.0 * Math.PI;
        return a;
    }

    public static double shortest_angular_distance(double from, double to) {
        double result = normalize_angle_positive(normalize_angle_positive(to) - normalize_angle_positive(from));

        if (result > Math.PI)
            // If the result > 180,
            // It's shorter the other way.
            result = -(2.0 * Math.PI - result);

        return normalize_angle(result);
    }
}
