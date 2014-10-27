package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;

import java.util.List;

public class RangeReading extends SensorReading {
    List<Double> readings;

    OrientedPoint<Double> m_pose;

    public RangeReading(RangeSensor m_sensor, double m_time) {
        super(m_sensor, m_time);
    }

    public RangeReading(int n_beams, double d, RangeSensor rs, double time) {
        super(rs, time);

    }

    public OrientedPoint<Double> getPose() {
        return m_pose;
    }

    public void setPose(OrientedPoint<Double> pose) {
        this.m_pose = pose;
    }

    int activeBeams(double density) {
        if (density == 0.) {
            return size();
        }
        int ab = 0;
        Point<Double> lastPoint = new Point<Double>(0.0, 0.0);
        int suppressed = 0;
        for (int i = 0; i < size(); i++) {
            RangeSensor rs = (RangeSensor) getSensor();
            Point<Double> lp = new Point<Double>(
                    Math.cos(rs.beams().get(i).pose.theta) * get(i),
                    Math.sin(rs.beams().get(i).pose.theta) * get(i));
            Point<Double> dp = Point.minus(lastPoint, lp);
            double distance = Math.sqrt(Point.mulD(dp, dp));
            if (distance < density) {
                suppressed++;
            } else {
                lastPoint = lp;
                ab++;
            }
        }
        return ab;
    }
}
