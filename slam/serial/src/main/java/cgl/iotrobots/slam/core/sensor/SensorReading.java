package cgl.iotrobots.slam.core.sensor;

import java.util.ArrayList;

public class SensorReading extends ArrayList<Double> {
    double m_time;
    Sensor m_sensor;

    public SensorReading(Sensor m_sensor, double m_time) {
        this.m_sensor = m_sensor;
        this.m_time = m_time;
    }

    public double getTime() {
        return m_time;
    }

    public Sensor getSensor() {
        return m_sensor;
    }

    public void setTime(double time) {
        this.m_time = time;
    }
}
