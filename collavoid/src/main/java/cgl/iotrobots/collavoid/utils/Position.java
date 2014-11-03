package cgl.iotrobots.collavoid.utils;

/**
 * Created by hjh on 10/31/14.
 */
public class Position {

    private Vector2 pos;
    private double heading;//heading

    public Position(double x, double y) {
        this.pos.setX(x);
        this.pos.setY(y);
    }

    public Position(double x, double y, double heading) {
        this.pos.setX(x);
        this.pos.setY(y);
        this.heading = heading;
    }

    public Position(Vector2 p) {
        this.pos.setVector2(p);
    }

    public Position(Vector2 p, double heading) {
        this.pos.setVector2(p);
        this.heading = heading;
    }

    public Vector2 getPos(){
        return new Vector2(this.pos);
    }

    public double getHeading(){
        return this.heading;
    }



}
