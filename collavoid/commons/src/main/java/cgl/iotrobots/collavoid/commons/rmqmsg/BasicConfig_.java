package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

/**
 * Created by hjh on 1/1/15.
 */
public class BasicConfig_ implements Serializable {
    private String robotName;
    private long seq;
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

    public String getRobotName() {
        return robotName;
    }

    public long getSeq() {
        return seq;
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

    public void setRobotName(String name) {
        this.robotName = name;
    }

    public void setSeq(long Seq) {
        this.seq = Seq;
    }

    @Override
    public String toString() {
        return "{start: " + start.toString() +
                "; goal: " + goal.toString() +
                "; pose share frequency: " + publisMeFreq +
                "; control frequency " + controlFreq + "}";
    }

}
