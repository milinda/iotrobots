package cgl.iotrobots.collavoid.commons;

import java.io.Serializable;

public class Vector3d_ implements Serializable {
    private double x;
    private double y;
    private double z;

    public Vector3d_() {
        x = 0;
        y = 0;
        z = 0;
    }

    public Vector3d_(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
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

    @Override
    public String toString() {
        return "(" +
                "," + x +
                "," + y +
                "," + z +
                ')';
    }
}
