package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;
import nav_msgs.Odometry;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Odometry_ implements Serializable {

    private Header_ Header;

    private String ChildFrameId;

    private Pose_ Pose;

    private Twist_ Twist;

    public Odometry_() {

    }

    public Odometry_(Odometry_ odom) {
        setTwist(odom.getTwist());
        setHeader(odom.getHeader());
        setChildFrameId(odom.getChildFrameId());
        setPose(odom.getPose());
    }

    public Odometry_(Pose_ pose, Twist_ twist) {
        setPose(pose);
        setTwist(twist);
    }


    public void setHeader(Header_ header) {
        Header = new Header_(header);
        Twist.setHeader(header);
    }

    public void setHeader(String frameId, long stamp) {
        Header.setFrameId(frameId);
        Header.setStamp(stamp);
    }

    public void setChildFrameId(String childFrameId) {
        ChildFrameId = childFrameId;
    }

    public void setPose(Pose_ pose) {
        Pose = new Pose_(pose);
    }

    public void setTwist(Twist_ twist) {
        Twist = new Twist_(twist);
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

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

}
