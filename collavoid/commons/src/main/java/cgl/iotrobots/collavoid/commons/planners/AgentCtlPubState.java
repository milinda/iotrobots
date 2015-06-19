package cgl.iotrobots.collavoid.commons.planners;


import java.io.Serializable;

public class AgentCtlPubState implements Serializable {
    public long lastTimeMePublished;
    public long lastTimeControlled;
    public double publishMePeriod;
    public double controlPeriod;
    public boolean publishing = false;
    public boolean controlling = false;
    public long seq;

    public AgentCtlPubState() {
        lastTimeMePublished = System.currentTimeMillis();
        lastTimeControlled = System.currentTimeMillis();
        publishMePeriod = 1.0 / Parameters.PUBLISH_ME_FREQUENCY;
        controlPeriod = 1.0 / Parameters.CONTROLLER_FREQUENCY;
    }
}