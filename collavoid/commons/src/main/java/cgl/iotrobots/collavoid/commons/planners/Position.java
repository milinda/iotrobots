package cgl.iotrobots.collavoid.commons.planners;

import java.io.Serializable;

public class Position implements Serializable {

    private Vector2 pos = new Vector2();
    private double heading;//heading

    public Position() {
        this.pos.setX(0);
        this.pos.setY(0);
        this.heading = 0;
    }

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

    public Vector2 getPos() {
        return this.pos;
    }

    public double getHeading() {
        return this.heading;
    }

    public void setHeading(double hd) {
        this.heading = hd;
    }

    public void setPos(Vector2 pos) {
        this.pos = pos;
    }

    public Position copy() {
        Position res = new Position();
        res.setHeading(heading);
        res.setPos(pos.copy());
        return res;

    }


}
