package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;
import cgl.iotrobots.slam.core.utils.Point;

import java.util.ArrayList;
import java.util.List;

public class RangeReading extends SensorReading {
    List<Double> readings;

    OrientedPoint<Double> m_pose;

    public RangeReading(RangeSensor m_sensor, double m_time) {
        super(m_sensor, m_time);
    }

    public RangeReading(int n_beams, Double []d, RangeSensor rs, double time) {
        super(rs, time);
        clear();
        for (int i = 0; i < n_beams; i++) {
            add(d[i]);
        }
    }

    public OrientedPoint<Double> getPose() {
        return m_pose;
    }

    public void setPose(OrientedPoint<Double> pose) {
        this.m_pose = pose;
    }

    public int activeBeams(double density) {
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

    public List<Point<Double>> cartesianForm(double maxRange) {
        RangeSensor rangeSensor = (RangeSensor) getSensor();
        int m_beams = rangeSensor.beams().size();
        List<Point<Double>> cartesianPoints = new ArrayList<Point<Double>>(m_beams);
        double px, py, ps, pc;
        px = rangeSensor.getPose().x;
        py = rangeSensor.getPose().y;
        ps = Math.sin(rangeSensor.getPose().theta);
        pc = Math.cos(rangeSensor.getPose().theta);
        for (int i = 0; i < m_beams; i++) {
            double rho = get(i);
            double s = rangeSensor.beams().get(i).s;
            double c = rangeSensor.beams().get(i).c;
            if (rho >= maxRange) {
                cartesianPoints.add(i, new Point<Double>(0.0, 0.0));
            } else {
                Point<Double> p = new Point<Double>(rangeSensor.beams().get(i).pose.x + c * rho, rangeSensor.beams().get(i).pose.y + s * rho);
                cartesianPoints.add(new Point<Double>(px + pc * p.x - ps * p.y, py + ps * p.x + pc * p.y));
            }
        }
        return cartesianPoints;
    }

    public int rawView(double[] v, double density) {
        if (density == 0) {
            for (int i = 0; i < size(); i++)
                v[i] = this.get(i);
        } else {
            Point<Double> lastPoint = new Point<Double>(0.0, 0.0);
            int suppressed = 0;
            for (int i = 0; i < size(); i++) {
                RangeSensor rs = (RangeSensor) getSensor();
                Point<Double> lp = new Point<Double>(
                        Math.cos(rs.beams().get(i).pose.theta) * this.get(i),
                        Math.sin(rs.beams().get(i).pose.theta) * this.get(i));
                Point<Double> dp = Point.minus(lastPoint, lp);
                double distance = Math.sqrt(Point.mulD(dp, dp));
                if (distance < density) {
                    //				v[i]=MAXDOUBLE;
                    v[i] = Double.MAX_VALUE;
                    suppressed++;
                } else {
                    lastPoint = lp;
                    v[i] = this.get(i);
                }
            }
        }
        return size();
    }
}
