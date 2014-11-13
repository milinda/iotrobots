package cgl.iotrobots.collavoid.utils;

import cgl.iotrobots.collavoid.ROSAgent.Agent;
import geometry_msgs.Pose;
import geometry_msgs.PoseStamped;
;


public class PoseWeighted {
    private double w;
    private PoseStamped poseStamped;

    public PoseWeighted(double w,PoseStamped pose){
        this.setW(w);
        this.setPoseStamped(pose);
    }

    public void setW(double w){
        this.w=w;
    }

    public void setPoseStamped(PoseStamped pose){
        this.poseStamped= Agent.messageFactory.newFromType(PoseStamped._TYPE);
        this.poseStamped.setPose(pose.getPose());
        this.poseStamped.setHeader(pose.getHeader());
    }

    public double getW(){
        return this.w;
    }

    public PoseStamped getPoseStamped() {
        PoseStamped ps=Agent.messageFactory.newFromType(PoseStamped._TYPE);
        ps.setHeader(this.poseStamped.getHeader());
        ps.setPose(this.poseStamped.getPose());
        return ps;
    }
//    public Pose getPose(){
//        return this.poseStamped.getPose();
//    }
}
