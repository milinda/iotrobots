package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

public class Vector3d_ implements Serializable {
    private double x;
    private double y;
    private double z;

    public Vector3d_() {
    }

    public Vector3d_(Vector3d_ v3d) {
        x = v3d.getX();
        y = v3d.getY();
        z = v3d.getZ();
    }

    public Vector3d_(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void scale(double s) {
        x = x * s;
        y = y * s;
        z = z * s;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getZ() {
        return z;
    }

    public double getY() {
        return y;
    }

    public double length() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3d_ copy() {
        return new Vector3d_(x, y, z);

    }

    @Override
    public String toString() {
        return "("
                + x +
                "," + y +
                "," + z +
                ')';
    }

}
