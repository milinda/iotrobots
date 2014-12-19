package cgl.iotrobots.collavoid.commons;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Twist_ implements Serializable {

    private Header_ Header;

    private Vector3d_ Angular;

    private Vector3d_ Linear;

    public Twist_(Vector3d_ angular, Vector3d_ linear) {
        Angular = angular;
        Linear = linear;
    }

    public Twist_() {
    }

    public void setAngular(Vector3d_ angular) {
        this.Angular = angular;
    }

    public void setLinear(Vector3d_ linear) {
        this.Linear = linear;
    }

    public Vector3d_ getAngular() {
        return Angular;
    }

    public Vector3d_ getLinear() {
        return Linear;
    }

    public void setHeader(String frameId, long stamp) {
        Header.setFrameId(frameId);
        Header.setStamp(stamp);
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    @Override
    public String toString() {
        return "{" +
                "angular=" + Angular +
                ", linear=" + Linear +
                '}';
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
