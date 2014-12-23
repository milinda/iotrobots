package cgl.iotrobots.collavoid.commons.planners;


public class Obstacle {
    private Vector2 begin;
    private Vector2 end;

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

    public Vector2 getBegin() {
        return new Vector2(this.begin);
    }

    public Vector2 getEnd() {
        return new Vector2(this.end);
    }
}
