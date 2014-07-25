package cgl.iotrobots.st.storm;

public class DecoderMessage {
    private byte []message;

    private long time;

    public DecoderMessage(byte[] message, long time) {
        this.message = message;
        this.time = time;
    }

    public void setMessage(byte[] message) {
        this.message = message;
    }

    public void setTime(long time) {
        this.time = time;
    }

    public byte[] getMessage() {
        return message;
    }

    public long getTime() {
        return time;
    }
}
