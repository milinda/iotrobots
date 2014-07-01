package cgl.iotrobots.st.commons;

public class Control {
    private double []position;

    private boolean hover;

    public Control(double [] position, boolean hover) {
        this.position = position;
        this.hover = hover;
    }

    public Control() {
    }

    public boolean isHover() {
        return hover;
    }

    public void setHover(boolean hover) {
        this.hover = hover;
    }

    public double[] getPosition() {
        return position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }
}
