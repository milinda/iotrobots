package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class OdometryReading extends SensorReading {
    OrientedPoint m_pose;
    OrientedPoint m_speed;
    OrientedPoint m_acceleration;

    public OdometryReading(Sensor m_sensor, double m_time) {
        super(m_sensor, m_time);
    }
}
