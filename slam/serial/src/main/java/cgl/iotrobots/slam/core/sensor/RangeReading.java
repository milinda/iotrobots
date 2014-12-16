package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;

import java.util.ArrayList;
import java.util.List;

public class RangeReading extends SensorReading {
    private DoubleOrientedPoint pose;

    public RangeReading() {
    }

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

    public DoubleOrientedPoint getPose() {
        return pose;
    }

    public void setPose(DoubleOrientedPoint pose) {
        this.pose = pose;
    }

    public int activeBeams(double density) {
        if (density == 0.) {
            return size();
        }
        int ab = 0;
        DoublePoint lastPoint = new DoublePoint(0.0, 0.0);
        int suppressed = 0;
        for (int i = 0; i < size(); i++) {
            RangeSensor rs = (RangeSensor) getSensor();
            DoublePoint lp = new DoublePoint(
                    Math.cos(rs.beams().get(i).pose.theta) * get(i),
                    Math.sin(rs.beams().get(i).pose.theta) * get(i));
            DoublePoint dp = DoublePoint.minus(lastPoint, lp);
            double distance = Math.sqrt(DoublePoint.mulD(dp, dp));
            if (distance < density) {
                suppressed++;
            } else {
                lastPoint = lp;
                ab++;
            }
        }
        return ab;
    }

    public List<DoublePoint> cartesianForm(double maxRange) {
        RangeSensor rangeSensor = (RangeSensor) getSensor();
        int m_beams = rangeSensor.beams().size();
        List<DoublePoint> cartesianPoints = new ArrayList<DoublePoint>(m_beams);
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
                cartesianPoints.add(i, new DoublePoint(0.0, 0.0));
            } else {
                DoublePoint p = new DoublePoint(rangeSensor.beams().get(i).pose.x + c * rho, rangeSensor.beams().get(i).pose.y + s * rho);
                cartesianPoints.add(new DoublePoint(px + pc * p.x - ps * p.y, py + ps * p.x + pc * p.y));
            }
        }
        return cartesianPoints;
    }

    public int rawView(double[] v, double density) {
        if (density == 0) {
            for (int i = 0; i < size(); i++)
                v[i] = this.get(i);
        } else {
            DoublePoint lastPoint = new DoublePoint(0.0, 0.0);
            int suppressed = 0;
            for (int i = 0; i < size(); i++) {
                RangeSensor rs = (RangeSensor) getSensor();
                DoublePoint lp = new DoublePoint(
                        Math.cos(rs.beams().get(i).pose.theta) * this.get(i),
                        Math.sin(rs.beams().get(i).pose.theta) * this.get(i));
                DoublePoint dp = DoublePoint.minus(lastPoint, lp);
                double distance = Math.sqrt(DoublePoint.mulD(dp, dp));
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
