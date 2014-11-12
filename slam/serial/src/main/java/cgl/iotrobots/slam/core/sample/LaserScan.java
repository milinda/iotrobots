package cgl.iotrobots.slam.core.sample;

import java.util.ArrayList;
import java.util.List;

public class LaserScan {
    public long timestamp;

    public double range_max;

    public double range_min;

    public double angle_min;

    public double angle_max;

    public double angle_increment;

    public List<Double> ranges = new ArrayList<Double>();
}
