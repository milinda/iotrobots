package cgl.iotrobots.slam.core.sensor;

import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class OdometryReading extends SensorReading {
    OrientedPoint m_pose;
    OrientedPoint m_speed;
    OrientedPoint m_acceleration;

    public OdometryReading(OdometrySensor m_sensor, double m_time) {
        super(m_sensor, m_time);
    }

    public OrientedPoint getPose() {
        return m_pose;
    }

    public OrientedPoint getSpeed() {
        return m_speed;
    }

    public OrientedPoint getAcceleration() {
        return m_acceleration;
    }

    public void setPose(OrientedPoint m_pose) {
        this.m_pose = m_pose;
    }

    public void setSpeed(OrientedPoint m_speed) {
        this.m_speed = m_speed;
    }

    public void setAcceleration(OrientedPoint m_acceleration) {
        this.m_acceleration = m_acceleration;
    }
}
