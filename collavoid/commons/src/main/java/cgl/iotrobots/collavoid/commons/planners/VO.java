package cgl.iotrobots.collavoid.commons.planners;

import java.io.Serializable;

public class VO implements Serializable {
    //counter clock wise direction
    private Vector2 point;
    private Vector2 relativePosition;
    private double combinedRadius;

    private Vector2 leftLegDir;
    private Vector2 rightLegDir;

    private Vector2 truncLineCenter;
    private Vector2 truncLeft;
    private Vector2 truncRight;

    private String type;

    public VO() {

    }

    public void setPoint(Vector2 v) {
        this.point = new Vector2(v);
    }

    public void setRelativePosition(Vector2 v) {
        this.relativePosition = new Vector2(v);
    }

    public void setCombinedRadius(double r) {
        this.combinedRadius = r;
    }

    public void setLeftLegDir(Vector2 v) {
        this.leftLegDir = new Vector2(v);
    }

    public void setRightLegDir(Vector2 v) {
        this.rightLegDir = new Vector2(v);
    }

    public void setTruncLineCenter(Vector2 v) {
        this.truncLineCenter = new Vector2(v);
    }

    public void setTruncLeft(Vector2 v) {
        this.truncLeft = new Vector2(v);
    }

    public void setTruncRight(Vector2 v) {
        this.truncRight = new Vector2(v);
    }

    public void setType(String type) {
        this.type = type;
    }

    public Vector2 getPoint() {
        return new Vector2(this.point);
    }

    public Vector2 getRelativePosition() {
        return new Vector2(this.relativePosition);
    }

    public double getCombinedRadius() {
        return this.combinedRadius;
    }

    public Vector2 getLeftLegDir() {
        return new Vector2(this.leftLegDir);
    }

    public Vector2 getRightLegDir() {
        return new Vector2(this.rightLegDir);
    }

    public Vector2 getTruncLineCenter() {
        return new Vector2(this.truncLineCenter);
    }

    public Vector2 getTruncLeft() {
        return new Vector2(this.truncLeft);
    }

    public Vector2 getTruncRight() {
        return new Vector2(this.truncRight);
    }

    public String getType() {
        return type;
    }
}
