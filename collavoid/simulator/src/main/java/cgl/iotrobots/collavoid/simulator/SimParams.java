package cgl.iotrobots.collavoid.simulator;

public class SimParams {
    //sensor parameters
    // scanner angle range
    public static final double SCAN_ANGLE_RANGE = 180.0; //in degree
    // distance range,in meter
    public static final double SCAN_MAX_RANGE = 3.5;
    // public static final double SCAN_MIN_RANGE = 1.2;
    public static final double SCAN_MIN_RANGE = 0;
    // resolutions, use laser sensor array to simulate laser scan and this is
    // the number of scan sensors
    public static final int SCAN_SENSOR_NB = 10;
    //scan update frequency
    public static final int SCAN_UPDATE_FREQ = 15;


}
