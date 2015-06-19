package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

public class Twist_ implements Serializable {

    private long Time;
    //test
    private long EmitTime;

    private Vector3d_ Angular = new Vector3d_();

    private Vector3d_ Linear = new Vector3d_();

    private long seq;

    private boolean goalReached;

    public Twist_() {
    }

    public void setEmitTime(long emitTime) {
        EmitTime = emitTime;
    }

    public void setSeq(long seq) {
        this.seq = seq;
    }

    public void setAngular(Vector3d_ angular) {
        this.Angular = new Vector3d_(angular);
    }

    public void setTime(long time) {
        Time = time;
    }

    public void setLinear(Vector3d_ linear) {
        this.Linear = new Vector3d_(linear);
    }

    public void setGoalReached(boolean goalReached) {
        this.goalReached = goalReached;
    }

    public Vector3d_ getAngular() {
        return Angular;
    }

    public long getTime() {
        return Time;
    }

    public Vector3d_ getLinear() {
        return Linear;
    }

    public long getEmitTime() {
        return EmitTime;
    }

    public boolean isGoalReached() {
        return goalReached;
    }

    public long getSeq() {
        return seq;
    }

    public Twist_ copy() {
        Twist_ twist_ = new Twist_();
        twist_.setAngular(Angular.copy());
        twist_.setLinear(Linear.copy());
        twist_.setGoalReached(goalReached);
        return twist_;
    }

    @Override
    public String toString() {
        return "{" +
                "angular=" + Angular +
                ", linear=" + Linear +
                '}';
    }


}
