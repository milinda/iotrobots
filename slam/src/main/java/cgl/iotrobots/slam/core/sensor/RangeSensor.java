package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

import java.util.ArrayList;
import java.util.List;

public class RangeSensor extends Sensor {
    OrientedPoint<Double> m_pose;
    List<Beam> m_beams = new ArrayList<Beam>();
    boolean newFormat;

    public RangeSensor(String name, int beams_num, double res, OrientedPoint<Double> position, double span, double maxrange) {
        super(name);
        this.m_pose = position;
        double angle = -.5 * res * beams_num;
        for (int i = 0; i < beams_num; i++, angle += res) {
            Beam beam = new Beam();
            beam.span = span;
            beam.pose.x = 0.0;
            beam.pose.y = 0.0;
            beam.pose.theta = angle;
            beam.maxRange = maxrange;
            m_beams.add(beam);
        }
        newFormat = false;
        updateBeamsLookup();
    }

    void updateBeamsLookup() {
        for (int i = 0; i < m_beams.size(); i++) {
            Beam beam = m_beams.get(i);
            beam.s = Math.sin(m_beams.get(i).pose.theta);
            beam.c = Math.cos(m_beams.get(i).pose.theta);
        }
    }

    public class Beam {
        OrientedPoint<Double> pose;
        double span;
        double maxRange;
        double s, c;
    }

    public List<Beam> beams() {
        return m_beams;
    }

    public OrientedPoint<Double> getPose() {
        return m_pose;
    }
}
