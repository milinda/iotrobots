package cgl.iotrobots.collavoid.utils;


public class ConvexHullPoint {

    private Vector2 point;
    private double weight;
    private int index;
    private int orig_index;

    public void setPoint(Vector2 v){
        this.point=new Vector2(v);
    }

    public Vector2 getPoint(){
        return new Vector2(this.point);
    }

}
