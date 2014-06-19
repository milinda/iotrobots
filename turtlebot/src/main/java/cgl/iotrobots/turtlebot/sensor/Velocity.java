package cgl.iotrobots.turtlebot.sensor;

public class Velocity {
    public enum Type {
        ANGULAR,
        LINEAR
    }

    private double x;

    private double y;

    private double z;

    private Type type;

    public Velocity(double x, double y, double z, Type type) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.type = type;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public Type getType() {
        return type;
    }
}
