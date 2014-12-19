package cgl.iotrobots.collavoid.simulator;

public class SimParams {
    // For simulator

    //robot parameters
    //robot number, in meter
    public static final int ROBOT_NB = 4;
    //robot radius, not used, use footprint radius from ROSAgent instead.
    public static final double ROBOT_RADIUS = 0.17;
    //robot position radius, robot are evenly distributed around a circle and
    // heading toward the center
    public static final double POSE_RADIUS = 4;

    //sensor parameters
    // scanner angle range
    public static final double SCAN_ANGLE_RANGE = 57.0; //in degree
    // distance range,in meter
    public static final double SCAN_MAX_RANGE = 3.5;
    public static final double SCAN_MIN_RANGE = 1.2;
    // resolutions, use laser sensor array to simulate laser scan and this is
    // the number of scan sensors
    public static final int SCAN_SENSOR_NB = 100;
    //scan update frequency
    public static final int SCAN_UPDATE_FREQ = 20;


}
