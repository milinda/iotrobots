package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

public class PoseShareMsg_ implements Serializable {

    private Header_ Header = new Header_();

    private String RobotId = "";

    private double Radius;

    private boolean HoloRobot;

    private boolean Controlled;

    private Vector3d_ HolonomicVelocity = new Vector3d_();

    private Pose_ Pose = new Pose_();

    private Twist_ Twist = new Twist_();

    private List<Vector3d_> FootPrint = new ArrayList<Vector3d_>();    // footprint minkowski

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

    public List<Vector3d_> getFootPrint() {
        return FootPrint;
    }

    public Pose_ getPose() {
        return Pose;
    }

    public String getRobotId() {
        return RobotId;
    }

    public Twist_ getTwist() {
        return Twist;
    }

    public Vector3d_ getHolonomicVelocity() {
        return HolonomicVelocity;
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

    public void setFootPrint(List<Vector3d_> footPrint) {
        FootPrint = footPrint;
    }

    public void setHolonomicVelocity(Vector3d_ holonomicVelocity) {
        HolonomicVelocity = holonomicVelocity;
    }

    public void setHoloRobot(boolean holoRobot) {
        HoloRobot = holoRobot;
    }

    public void setRadius(double radius) {
        Radius = radius;
    }

    public void setRobotId(String robotId) {
        RobotId = robotId;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
