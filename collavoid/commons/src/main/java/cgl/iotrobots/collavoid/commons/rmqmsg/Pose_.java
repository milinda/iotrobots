package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;
import geometry_msgs.Pose;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Pose_ implements Serializable {

    private Header_ Header;

    private Vector3d_ Position;

    private Vector4d_ Orientation;

    public Pose_() {
        Header = new Header_();
        Position = new Vector3d_();
        Orientation = new Vector4d_();
    }

    public Pose_(Pose_ pose) {
        Header = new Header_(pose.getHeader());
        Position = new Vector3d_(pose.getPosition());
        Orientation = new Vector4d_(pose.getOrientation());
    }

    public Pose_(Vector3d_ pos, Vector4d_ ori) {
        this.Position = new Vector3d_(pos);
        this.Orientation = new Vector4d_(ori);
    }

    public Header_ getHeader() {
        return Header;
    }

    public Vector3d_ getPosition() {
        return Position;
    }

    public Vector4d_ getOrientation() {
        return Orientation;
    }

    public void setPosition(Vector3d_ position) {
        Position = new Vector3d_(position);
    }

    public void setOrientation(Vector4d_ orientation) {
        Orientation = new Vector4d_(orientation);
    }

    public void setHeader(Header_ header) {
        Header = new Header_(header);
    }

    public void setPose(Pose_ pose) {
        setOrientation(pose.getOrientation());
        setPosition(pose.getPosition());
        setHeader(pose.getHeader());
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

}
