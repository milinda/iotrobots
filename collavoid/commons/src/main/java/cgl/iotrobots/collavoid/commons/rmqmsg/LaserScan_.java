package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by hjh on 2/6/15.
 */
public class LaserScan_ implements Serializable {
    private Header_ Header = new Header_();
    private double Angle_min;
    private double Angle_max;
    private double Angle_increment;
    private double Time_increment;
    private double Scan_time;
    private double Range_min;
    private double Range_max;
    private List<Double> Ranges = new ArrayList<Double>();
    private double[] Intensities;

    public double getAngle_increment() {
        return Angle_increment;
    }

    public double getAngle_max() {
        return Angle_max;
    }

    public double getAngle_min() {
        return Angle_min;
    }

    public double getRange_max() {
        return Range_max;
    }

    public double getRange_min() {
        return Range_min;
    }

    public double getScan_time() {
        return Scan_time;
    }

    public double getTime_increment() {
        return Time_increment;
    }

    public double[] getIntensities() {
        return Intensities;
    }

    public List<Double> getRanges() {
        return Ranges;
    }

    public Header_ getHeader() {
        return Header;
    }

    public void setAngle_increment(double angle_increment) {
        Angle_increment = angle_increment;
    }

    public void setAngle_max(double angle_max) {
        Angle_max = angle_max;
    }

    public void setAngle_min(double angle_min) {
        Angle_min = angle_min;
    }

    public void setHeader(Header_ header) {
        Header = header;
    }

    public void setIntensities(double[] intensities) {
        Intensities = intensities;
    }

    public void setRange_max(double range_max) {
        Range_max = range_max;
    }

    public void setRange_min(double range_min) {
        Range_min = range_min;
    }

    public void setRanges(List<Double> ranges) {
        Ranges = ranges;
    }

    public void setScan_time(double scan_time) {
        Scan_time = scan_time;
    }

    public void setTime_increment(double time_increment) {
        Time_increment = time_increment;
    }
}
