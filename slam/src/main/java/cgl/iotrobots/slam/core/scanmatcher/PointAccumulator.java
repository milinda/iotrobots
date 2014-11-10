package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.Point;

public class PointAccumulator {
    public static final int SIGHT_INC = 1;

    public Point<Double> acc = new Point<Double>(0.0, 0.0);

    public int n, visits;

    public PointAccumulator() {
    }

    public void update(boolean value, Point<Double> p) {
        if (value) {
            acc.x += p.x;
            acc.y += p.y;
            n++;
            visits += SIGHT_INC;
        } else {
            visits++;
        }
    }

    public double entropy() {
        if (visits == 0)
            return -Math.log(.5);
        if (n == visits || n == 0)
            return 0;
        double x = (double) n * SIGHT_INC / (double) visits;
        return -(x * Math.log(x) + (1 - x) * Math.log(1 - x));
    }

    public Point<Double> mean() {
        return new Point<Double>(1.0 /(acc.x * n), 1.0 /(n *acc.y));
    }

    void add(PointAccumulator p) {
        acc = new Point<Double>(acc.x + p.acc.x, acc.y + p.acc.y);
        n +=p.n;
        visits += p.visits;
    }

    public double doubleValue() {
        if (visits > 0) {
            System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAA");
        }
        return visits > 0 ? (double)n*SIGHT_INC/(double)visits:-1;
    }


}
