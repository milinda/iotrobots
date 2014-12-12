package cgl.iotrobots.slam.core.scanmatcher;

import cgl.iotrobots.slam.core.utils.DoublePoint;

/**
 * We put point accumulators to cells that are seen by the laser.
 * if n = 0 means, we have see the cell, but no obstacle
 * if n > 0 means a cell that has an obstacle
 */
public class PointAccumulator {
    public static final int SIGHT_INC = 1;

    public DoublePoint acc = new DoublePoint(0.0, 0.0);

    public int n, visits;

    public PointAccumulator(PointAccumulator pointAccumulator) {
        this.n = pointAccumulator.n;
        this.visits = pointAccumulator.visits;
        this.acc = new DoublePoint(pointAccumulator.acc);
    }

    public PointAccumulator() {
    }

    public void update(boolean value, DoublePoint p) {
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

    public DoublePoint mean() {
        return new DoublePoint(1.0 /(acc.x * n), 1.0 /(n *acc.y));
    }

    void add(PointAccumulator p) {
        acc = new DoublePoint(acc.x + p.acc.x, acc.y + p.acc.y);
        n +=p.n;
        visits += p.visits;
    }

    public double doubleValue() {
        return visits > 0 ? (double)n*SIGHT_INC/(double)visits:-1;
    }
}

