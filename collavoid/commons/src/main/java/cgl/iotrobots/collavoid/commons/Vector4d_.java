package cgl.iotrobots.collavoid.commons;

import java.io.Serializable;

public class Vector4d_ implements Serializable {
    private double x;
    private double y;
    private double z;
    private double w;

    public Vector4d_() {
        x = 0;
        y = 0;
        z = 0;
        w = 0;
    }

    public Vector4d_(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
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

    public void setW(double w) {
        this.w = w;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getW() {
        return w;
    }

    @Override
    public String toString() {
        return "(" +
                "," + x +
                "," + y +
                "," + z +
                "," + w +
                ')';
    }
}
