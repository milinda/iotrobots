package cgl.iotrobots.collavoid.commons;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;
import geometry_msgs.Pose;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Pose_ implements Serializable {

    private Vector3d_ Position;

    private Vector4d_ Orientation;

    public Pose_() {


    }

    public Pose_(Vector3d_ pos, Vector4d_ ori) {
        this.Position = pos;
        this.Orientation = ori;
    }

    public Pose_(Pose pose) {
        this.Position = new Vector3d_(
                pose.getPosition().getX(),
                pose.getPosition().getY(),
                pose.getPosition().getZ()
        );

        this.Orientation = new Vector4d_(
                pose.getOrientation().getX(),
                pose.getOrientation().getY(),
                pose.getOrientation().getZ(),
                pose.getOrientation().getW()
        );
    }

    public void setPosition(Vector3d_ position) {
        Position = position;
    }

    public void setOrientation(Vector4d_ orientation) {
        Orientation = orientation;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

}
