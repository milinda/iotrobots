package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

/**
 * Created by hjh on 12/20/14.
 */
public class PoseStamped_ implements Serializable {
    private Header_ Header = new Header_();

    private Pose_ Pose = new Pose_();

    public Header_ getHeader() {
        return Header;
    }

    public Pose_ getPose() {
        return Pose;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setPose(Pose_ pose) {
        Pose = pose;
    }

    public PoseStamped_ copy() {
        PoseStamped_ poseStamped_ = new PoseStamped_();
        poseStamped_.setHeader(Header.copy());
        poseStamped_.setPose(Pose.copy());
        return poseStamped_;

    }

    @Override
    public String toString() {
        return Header.toString() + Pose.toString();
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
