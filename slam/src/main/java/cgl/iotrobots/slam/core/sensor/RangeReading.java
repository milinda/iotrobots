package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class RangeReading extends SensorReading {
    OrientedPoint<Double> m_pose;

    public RangeReading(Sensor m_sensor, double m_time) {
        super(m_sensor, m_time);
    }

    public OrientedPoint<Double> getPose() {
        return m_pose;
    }
}
