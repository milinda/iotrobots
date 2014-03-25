package cgl.iotrobots.turtlebot.sensor;

public class Frame {
    int width;

    int height;

    int step;

    byte buffer[] = null;

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getStep() {
        return step;
    }

    public byte[] getBuffer() {
        return buffer;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public void setHeight(int height) {
        this.height = height;
    }

    public void setStep(int step) {
        this.step = step;
    }

    public void setBuffer(byte[] buffer) {
        this.buffer = buffer;
    }
}
