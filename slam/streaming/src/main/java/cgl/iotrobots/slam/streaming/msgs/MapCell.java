package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.utils.DoublePoint;

public class MapCell {
    private int x;
    private int y;

    private DoublePoint acc = new DoublePoint(0.0, 0.0);

    private int n, visits;

    public MapCell() {
    }

    public MapCell(int x, int y, DoublePoint acc, int n, int visits) {
        this.x = x;
        this.y = y;
        this.acc = acc;
        this.n = n;
        this.visits = visits;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public DoublePoint getAcc() {
        return acc;
    }

    public int getN() {
        return n;
    }

    public int getVisits() {
        return visits;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public void setAcc(DoublePoint acc) {
        this.acc = acc;
    }

    public void setN(int n) {
        this.n = n;
    }

    public void setVisits(int visits) {
        this.visits = visits;
    }
}
