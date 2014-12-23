package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Odometry_ implements Serializable {

    private Header_ Header = new Header_();

    private String ChildFrameId = "";

    private Pose_ Pose = new Pose_();

    private Twist_ Twist = new Twist_();

    public void setHeader(Header_ header) {
        Header = header;
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

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

    @Override
    public String toString() {
        return "{" +
                "Pose=" + Pose +
                ", Twist=" + Twist +
                '}';
    }
}
