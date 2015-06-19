package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

public class Odometry_ implements Serializable {

    private String id = "";
    private Header_ Header = new Header_();
    private String ChildFrameId = "";
    private Pose_ Pose = new Pose_();
    private Twist_ Twist = new Twist_();


    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setId(String id) {
        this.id = id;
    }

    public void setChildFrameId(String childFrameId) {
        ChildFrameId = childFrameId;
    }

    public void setPose(Pose_ pose) {
        Pose = pose;
    }

    public void setTwist(Twist_ twist) {
        Twist = twist;
    }

    public Header_ getHeader() {
        return Header;
    }

    public Pose_ getPose() {
        return Pose;
    }

    public String getChildFrameId() {
        return ChildFrameId;
    }

    public String getId() {
        return id;
    }

    public Twist_ getTwist() {
        return Twist;
    }

    public Odometry_ copy() {
        Odometry_ odometry_ = new Odometry_();
        odometry_.setHeader(Header.copy());
        odometry_.setChildFrameId(new String(ChildFrameId));
        odometry_.setTwist(Twist.copy());
        odometry_.setPose(Pose.copy());

        return odometry_;
    }

    @Override
    public String toString() {
        return "{" +
                "Pose=" + Pose +
                ", Twist=" + Twist +
                '}';
    }
}
