package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class Header_ implements Serializable {

    private String FrameId = "";

    private long Stamp;

    public Header_() {
    }

    public long getStamp() {
        return Stamp;
    }

    public String getFrameId() {
        return FrameId;
    }

    public void setFrameId(String frameId) {
        FrameId = new String(frameId);
    }

    public void setStamp(long stamp) {
        Stamp = stamp;
    }

    public byte[] toJSON() throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        mapper.writeValue(outputStream, this);
        return outputStream.toByteArray();
    }

}
