package cgl.iotrobots.slam.core.app;

import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

import java.util.ArrayList;
import java.util.List;

public class LaserScan {
    protected long timestamp;

    protected double rangeMax;

    protected double range_min;

    protected double angle_min;

    protected double angle_max;

    protected double angle_increment;

    protected List<Double> ranges = new ArrayList<Double>();

    private DoubleOrientedPoint pose;

    public DoubleOrientedPoint getPose() {
        return pose;
    }

    public void setPose(DoubleOrientedPoint pose) {
        this.pose = pose;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public double getRangeMax() {
        return rangeMax;
    }

    public double getRange_min() {
        return range_min;
    }

    public double getAngle_min() {
        return angle_min;
    }

    public double getAngle_max() {
        return angle_max;
    }

    public double getAngle_increment() {
        return angle_increment;
    }

    public List<Double> getRanges() {
        return ranges;
    }

    public void setTimestamp(long timestamp) {
        this.timestamp = timestamp;
    }

    public void setRangeMax(double rangeMax) {
        this.rangeMax = rangeMax;
    }

    public void setRange_min(double range_min) {
        this.range_min = range_min;
    }

    public void setAngle_min(double angle_min) {
        this.angle_min = angle_min;
    }

    public void setAngle_max(double angle_max) {
        this.angle_max = angle_max;
    }

    public void setAngle_increment(double angle_increment) {
        this.angle_increment = angle_increment;
    }

    public void setRanges(List<Double> ranges) {
        this.ranges = ranges;
    }

    public String getString() {
        StringBuilder builder = new StringBuilder();
        builder.append(rangeMax).append(",");
        builder.append(range_min).append(",");
        builder.append(angle_min).append(",");
        builder.append(angle_max).append(",");
        builder.append(angle_increment).append(",");
        builder.append(pose.getX()).append(",");
        builder.append(pose.getY()).append(",");
        builder.append(pose.getTheta());
        for (Double range : ranges) {
            builder.append(",").append(range);

        }
        return builder.toString();
    }

    public void loadFromString(String laserScan) {
        String vals[] = laserScan.split(",");
        rangeMax = Double.parseDouble(vals[0]);
        range_min = Double.parseDouble(vals[1]);
        angle_min = Double.parseDouble(vals[2]);
        angle_max = Double.parseDouble(vals[3]);
        angle_increment = Double.parseDouble(vals[4]);
        pose.setX(Double.parseDouble(vals[5]));
        pose.setY(Double.parseDouble(vals[6]));
        pose.setTheta(Double.parseDouble(vals[7]));
        for (int i = 8; i < vals.length; i++) {
            ranges.add(Double.parseDouble(vals[i]));
        }
    }
}
