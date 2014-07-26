package cgl.iotrobots.st.storm;

public class DecoderMessage {
    private byte []message;

    private String time;

    public DecoderMessage(byte[] message, String time) {
        this.message = message;
        this.time = time;
    }

    public void setMessage(byte[] message) {
        this.message = message;
    }

    public void setTime(String time) {
        this.time = time;
    }

    public byte[] getMessage() {
        return message;
    }

    public String getTime() {
        return time;
    }
}
