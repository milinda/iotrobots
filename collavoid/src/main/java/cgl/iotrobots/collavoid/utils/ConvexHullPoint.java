package cgl.iotrobots.collavoid.utils;


public class ConvexHullPoint {

    private double x;
    private double y;
    private double weight;
    private int index = 0;
    private int orig_index = 0;


    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPoint(Vector2 v) {
        setPoint(v.getX(), v.getY());
    }

    public void setWeight(double w){
        this.weight=w;
    }

    public void setIndex(int i){
        this.index=i;
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
