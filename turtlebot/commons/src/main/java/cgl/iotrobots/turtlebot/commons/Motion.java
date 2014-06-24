package cgl.iotrobots.turtlebot.commons;

public class Motion {
    private Velocity angular;
    private Velocity linear;

    public Motion(Velocity linear, Velocity angular) {
        this.angular = angular;
        this.linear = linear;
    }

    public Velocity getAngular() {
        return angular;
    }

    public Velocity getLinear() {
        return linear;
    }

    @Override
    public String toString() {
        return "{" +
                "angular=" + angular +
                ", linear=" + linear +
                '}';
    }
}
