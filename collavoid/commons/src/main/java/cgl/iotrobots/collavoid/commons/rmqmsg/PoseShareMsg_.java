package cgl.iotrobots.collavoid.commons.rmqmsg;

import cgl.iotrobots.collavoid.commons.planners.Vector2;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class PoseShareMsg_ implements Serializable {

    private Header_ Header = new Header_();

    public String id = "";

    private double Radius;//here we use footprint to calculate velocity, so this radius is the footprint radius

    private boolean HoloRobot;

    private boolean Controlled;

    public double PulishMePeriod;

    public long lastTimeMePublished;

    private Pose_ Pose = new Pose_();

    private Twist_ Twist = new Twist_();

    private List<Vector2> FootPrint_Minkowski = new ArrayList<Vector2>();    // footprint minkowski

    public List<Vector2> Footprint_original = new ArrayList<Vector2>();


    public Header_ getHeader() {
        return Header;
    }

    public double getRadius() {
        return Radius;
    }

    public boolean getHoloRobot() {
        return HoloRobot;
    }

    public boolean getControlled() {
        return Controlled;
    }

    public double getPulishMePeriod() {
        return PulishMePeriod;
    }

    public List<Vector2> getFootPrint_Minkowski() {
        return FootPrint_Minkowski;
    }

    public Pose_ getPose() {
        return Pose;
    }

    public String getId() {
        return id;
    }

    public Twist_ getTwist() {
        return Twist;
    }

    public List<Vector2> getFootprint_original() {
        return Footprint_original;
    }

    public long getLastTimeMePublished() {
        return lastTimeMePublished;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setTwist(Twist_ twist) {
        Twist = twist;
    }

    public void setPose(Pose_ pose) {
        Pose = pose;
    }

    public void setControlled(boolean controlled) {
        Controlled = controlled;
    }

    public void setFootPrint_Minkowski(List<Vector2> footPrint_Minkowski) {
        FootPrint_Minkowski = footPrint_Minkowski;
    }

    public void setHoloRobot(boolean holoRobot) {
        HoloRobot = holoRobot;
    }

    public void setRadius(double radius) {
        Radius = radius;
    }

    public void setId(String robotId) {
        id = robotId;
    }

    public void setPulishMePeriod(double pulishMePeriod) {
        this.PulishMePeriod = pulishMePeriod;
    }

    public void setFootprint_original(List<Vector2> footprint_original) {
        Footprint_original = footprint_original;
    }

    public void setLastTimeMePublished(long lastTimeMePublished) {
        this.lastTimeMePublished = lastTimeMePublished;
    }
}
