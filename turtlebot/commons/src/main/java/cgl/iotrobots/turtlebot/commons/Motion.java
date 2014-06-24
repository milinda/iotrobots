package cgl.iotrobots.turtlebot.commons;

import java.io.Serializable;

public class Motion implements Serializable {
    private Velocity angular;
    private Velocity linear;

    public Motion(Velocity linear, Velocity angular) {
        this.angular = angular;
        this.linear = linear;
    }

    public Motion() {
    }

    public void setAngular(Velocity angular) {
        this.angular = angular;
    }

    public void setLinear(Velocity linear) {
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
