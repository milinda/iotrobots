package cgl.iotrobots.st.storm;

public class DecoderMessage {
    private byte []message;

    private String time;

    private String sensorId;

    public DecoderMessage(byte[] message, String time, String sensorId) {
        this.message = message;
        this.time = time;
        this.sensorId = sensorId;
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

    public String getSensorId() {
        return sensorId;
    }
}
