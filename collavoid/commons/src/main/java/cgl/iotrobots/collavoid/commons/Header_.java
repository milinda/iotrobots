package cgl.iotrobots.collavoid.commons;

import java.io.Serializable;

public class Header_ implements Serializable {

    private String FrameId;

    private long Stamp;

    public Header_(String frameId, long stamp) {
        FrameId = frameId;
        Stamp = stamp;
    }

    public long getStamp() {
        return Stamp;
    }

    public String getFrameId() {
        return FrameId;
    }

    public void setFrameId(String frameId) {
        FrameId = frameId;
    }

    public void setStamp(long stamp) {
        Stamp = stamp;
    }
}
