package cgl.iotrobots.slam.core.utils;

import cgl.iotrobots.slam.core.app.LaserScan;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Utils {
    private static Logger LOG = LoggerFactory.getLogger(Utils.class);

    public static Double[] getDoubles(LaserScan scan, double gspLaserAngleIncrement) {
        // GMapping wants an array of doubles...
        Double[] ranges_double = new Double[scan.ranges.size()];
        // If the angle increment is negative, we have to invert the order of the readings.
        if (gspLaserAngleIncrement < 0) {
            LOG.debug("Inverting scan");
            int num_ranges = scan.ranges.size();
            for (int i = 0; i < num_ranges; i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.ranges.get(i) < scan.range_min) {
                    ranges_double[i] = scan.rangeMax;
                } else {
                    ranges_double[i] = scan.ranges.get(num_ranges - i - 1);
                }
            }
        } else {
            for (int i = 0; i < scan.ranges.size(); i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.ranges.get(i) < scan.range_min) {
                    ranges_double[i] = scan.rangeMax;
                } else {
                    ranges_double[i] = scan.ranges.get(i);
                }
            }
        }
        return ranges_double;
    }
}
