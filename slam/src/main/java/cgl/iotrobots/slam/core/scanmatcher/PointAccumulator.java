package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.Point;

public class PointAccumulator {
    public static final int SIGHT_INC = 1;

    public Point<Double> acc;

    public int n, visits;

    void update(boolean value, Point<Double> p) {
        if (value) {
            acc.x += p.x;
            acc.y += p.y;
            n++;
            visits += SIGHT_INC;
        } else {
            visits++;
        }
    }

    double entropy() {
        if (visits == 0)
            return -Math.log(.5);
        if (n == visits || n == 0)
            return 0;
        double x = (double) n * SIGHT_INC / (double) visits;
        return -(x * Math.log(x) + (1 - x) * Math.log(1 - x));
    }
}
