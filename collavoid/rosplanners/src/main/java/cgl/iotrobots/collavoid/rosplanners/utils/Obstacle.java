package cgl.iotrobots.collavoid.rosplanners.utils;

import org.ros.message.Time;


public class Obstacle {
    private Vector2 begin;
    private Vector2 end;
    private Time lastSeen;

    public Obstacle(Vector2 p1, Vector2 p2) {
        this.begin = new Vector2(p1.getX(), p1.getY());
        this.end = new Vector2(p2.getX(), p2.getY());
    }

    public void setBothPoints(Vector2 p1, Vector2 p2) {
        this.begin = new Vector2(p1);
        this.end = new Vector2(p2);
    }

    public void setBegin(Vector2 v) {
        this.begin = new Vector2(v);
    }

    public void setEnd(Vector2 v) {
        this.end = new Vector2(v);
    }

    public void setTime(Time t) {
        lastSeen = new Time(t);
    }

    public Vector2 getBegin() {
        return new Vector2(this.begin);
    }

    public Vector2 getEnd() {
        return new Vector2(this.end);
    }
}
