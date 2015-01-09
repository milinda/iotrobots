package cgl.iotrobots.collavoid.commons.rmqmsg;

import cgl.iotrobots.collavoid.commons.planners.Vector2;
import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class PoseShareMsg_ implements Serializable {

    private Header_ Header = new Header_();

    private String Name = "";

    private double Radius;

    private boolean HoloRobot;

    private boolean Controlled;

    private double ControlPeriod;

    private Pose_ Pose = new Pose_();

    private Twist_ Twist = new Twist_();

    private List<Vector2> FootPrint_Minkowski = new ArrayList<Vector2>();    // footprint minkowski


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

    public double getControlPeriod() {
        return ControlPeriod;
    }

    public List<Vector2> getFootPrint_Minkowski() {
        return FootPrint_Minkowski;
    }

    public Pose_ getPose() {
        return Pose;
    }

    public String getName() {
        return Name;
    }

    public Twist_ getTwist() {
        return Twist;
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

    public void setName(String robotId) {
        Name = robotId;
    }

    public void setControlPeriod(double controlPeriod) {
        this.ControlPeriod = controlPeriod;
    }

}
