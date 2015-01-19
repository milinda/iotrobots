package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

/**
 * Created by hjh on 1/1/15.
 */
public class BaseConfig_ implements Serializable {
    private String id;
    private long time;
    private PoseStamped_ start;
    private PoseStamped_ goal;
    private double publisMeFreq;
    private double controlFreq;

    public PoseStamped_ getGoal() {
        return goal;
    }

    public PoseStamped_ getStart() {
        return start;
    }

    public double getControlFreq() {
        return controlFreq;
    }

    public double getPublisMeFreq() {
        return publisMeFreq;
    }

    public long getTime() {
        return time;
    }

    public String getId() {
        return id;
    }

    public void setGoal(PoseStamped_ goal) {
        this.goal = goal;
    }

    public void setStart(PoseStamped_ start) {
        this.start = start;
    }

    public void setControlFreq(double controlFreq) {
        this.controlFreq = controlFreq;
    }

    public void setPublisMeFreq(double publisMeFreq) {
        this.publisMeFreq = publisMeFreq;
    }

    public void setTime(long time) {
        this.time = time;
    }

    public void setId(String id) {
        this.id = id;
    }

    @Override
    public String toString() {
        return "{start: " + start.toString() +
                "; goal: " + goal.toString() +
                "; pose share frequency: " + publisMeFreq +
                "; control frequency " + controlFreq + "}";
    }

}
