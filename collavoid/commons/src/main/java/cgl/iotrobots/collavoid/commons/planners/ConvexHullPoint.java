package cgl.iotrobots.collavoid.commons.planners;

import java.io.Serializable;

public class ConvexHullPoint implements Serializable {

    private double x;
    private double y;
    private double weight;
    private int index = 0;
    private int orig_index = 0;

    public ConvexHullPoint() {
        x = 0;
        y = 0;
        weight = 0;
    }

    public ConvexHullPoint(double x, double y) {
        this.x = x;
        this.y = y;
        weight = 0;
    }

    public ConvexHullPoint(double x, double y, double w) {
        this.x = x;
        this.y = y;
        weight = w;
    }

    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(Vector2 v) {
        setPoint(v.getX(), v.getY());
    }

    public void setWeight(double w) {
        this.weight = w;
    }

    public void setIndex(int i) {
        this.index = i;
    }

    public void setOrig_index(int orig_index) {
        this.orig_index = orig_index;
    }

    public double getY() {
        return y;
    }

    public double getX() {
        return x;
    }

    public double getWeight() {
        return weight;
    }

    public int getIndex() {
        return index;
    }
}
