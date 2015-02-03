package cgl.iotrobots.slam.core.utils;

import cgl.iotrobots.slam.core.app.LaserScan;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Utils {
    private static Logger LOG = LoggerFactory.getLogger(Utils.class);

    public static Double[] getDoubles(LaserScan scan, double gspLaserAngleIncrement) {
        // GMapping wants an array of doubles...
        Double[] ranges_double = new Double[scan.getRanges().size()];
        // If the angle increment is negative, we have to invert the order of the readings.
        if (gspLaserAngleIncrement < 0) {
            LOG.debug("Inverting scan");
            int num_ranges = scan.getRanges().size();
            for (int i = 0; i < num_ranges; i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.getRanges().get(i) < scan.getRangeMin()) {
                    ranges_double[i] = scan.getRangeMax();
                } else {
                    ranges_double[i] = scan.getRanges().get(num_ranges - i - 1);
                }
            }
        } else {
            for (int i = 0; i < scan.getRanges().size(); i++) {
                // Must filter out short readings, because the mapper won't
                if (scan.getRanges().get(i) < scan.getRangeMin()) {
                    ranges_double[i] = scan.getRangeMax();
                } else {
                    ranges_double[i] = scan.getRanges().get(i);
                }
            }
        }
        return ranges_double;
    }
}
