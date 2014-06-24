package cgl.iotrobots.turtlebot.commons;

import java.io.Serializable;

public class Velocity implements Serializable {
    private double x;

    private double y;

    private double z;

    public Velocity(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
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

    @Override
    public String toString() {
        return "{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }
}
