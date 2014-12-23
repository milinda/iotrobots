package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Pose_ implements Serializable {

    private Vector3d_ Position = new Vector3d_();

    private Vector4d_ Orientation = new Vector4d_();

    public Pose_() {
    }

    public Vector3d_ getPosition() {
        return Position;
    }

    public Vector4d_ getOrientation() {
        return Orientation;
    }

    public void setPosition(Vector3d_ position) {
        Position = position;
    }

    public void setOrientation(Vector4d_ orientation) {
        Orientation = orientation;
    }

    public Pose_ copy() {
        Pose_ pose_ = new Pose_();
        pose_.setPosition(Position.copy());
        pose_.setOrientation(Orientation.copy());
        return pose_;
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
                "Position=" + Position +
                ", Orientation=" + Orientation +
                '}';
    }
}
