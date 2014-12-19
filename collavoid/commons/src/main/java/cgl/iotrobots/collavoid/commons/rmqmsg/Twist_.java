package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Twist_ implements Serializable {

    private Header_ Header;

    private Vector3d_ Angular;

    private Vector3d_ Linear;

    public Twist_(Twist_ t) {
        setHeader(t.getHeader());
        setAngular(t.getAngular());
        setLinear(t.getLinear());
    }

    public Twist_(Vector3d_ angular, Vector3d_ linear) {
        Angular = new Vector3d_(angular);
        Linear = new Vector3d_(linear);
    }

    public Twist_() {
        Angular = new Vector3d_(0, 0, 0);
        Linear = new Vector3d_(0, 0, 0);
    }

    public void setAngular(Vector3d_ angular) {
        this.Angular = new Vector3d_(angular);
    }

    public void setLinear(Vector3d_ linear) {
        this.Linear = new Vector3d_(linear);
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
        Header = new Header_(header);
    }

    public Header_ getHeader() {
        return Header;
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
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }
}
