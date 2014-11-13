package cgl.iotrobots.collavoid.utils;


public class ConvexHullPoint {

    private Vector2 point;
    private double weight;
    private int index;
    private int orig_index;


    public void setPoint(Vector2 v){
        this.point=new Vector2(v);
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

    public Vector2 getPoint(){
        return new Vector2(this.point);
    }

    public double getWeight() {
        return weight;
    }

    public int getIndex() {
        return index;
    }
}
