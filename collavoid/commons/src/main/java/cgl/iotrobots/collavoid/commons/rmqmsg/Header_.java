package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;

public class Header_ implements Serializable {

    private String FrameId = "";

    private long Stamp;

    private long seq;

    public Header_() {
    }

    public long getStamp() {
        return Stamp;
    }

    public long getSeq() {
        return seq;
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

    public void setSeq(long seq) {
        this.seq = seq;
    }

    public Header_ copy() {
        Header_ header_ = new Header_();
        header_.setStamp(Stamp);
        header_.setFrameId(FrameId);
        header_.setSeq(seq);
        return header_;
    }

    @Override
    public String toString() {
        return "{FrameId:" + FrameId + "," + "Stamp:" + Stamp + "}";
    }

}
