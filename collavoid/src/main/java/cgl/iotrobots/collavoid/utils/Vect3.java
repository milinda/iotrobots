package cgl.iotrobots.collavoid.utils;

/**
 * Created by hjh on 10/31/14.
 */
public class Vect3 extends Vector2 {

    public double x;
    public double y;
    public double theta;//heading

    public Vect3(double x, double y) {
        super(x, y);
    }

    public Vect3(double x, double y, double theta) {
        super(x, y);
        this.theta = theta;
    }

    public Vect3(Vector2 p) {
        super(p.x, p.y);
        if (p instanceof Vect3) {
            theta = (double) ((Vect3) p).theta;
        }
    }

    public Vector2 getVector2() {
        return new Vector2(this.x, this.y);
    }
}
