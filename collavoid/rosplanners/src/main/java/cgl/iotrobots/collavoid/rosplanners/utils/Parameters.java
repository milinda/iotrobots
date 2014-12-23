package cgl.iotrobots.collavoid.rosplanners.utils;

public class Parameters {
    public static final String BASE_FRAME_SUFFIX = "_base";
    public static final String GLOBAL_FRAME = "map";

    // local planner parameters
    public static final double ROT_STOPPED_VELOCITY = 0.02;
    public static final double TRANS_STOPPED_VELOCITY = 0.01;
    public static final double YAW_GOAL_TOLERANCE = 0.10;
    public static final double XY_GOAL_TOLERANCE = 0.10;
    public static final boolean LATCH_XY_GOAL_TOLERANCE = true;
    public static final boolean IGNORE_GOAL_YAW = false;

    // agent parameters
    public static final boolean HOLO_ROBOT = false;
    public static final double WHEEL_BASE = 0.25;

    public static final double MAX_VEL_X = 0.5;
    public static final double MIN_VEL_X = 0.05;
    public static final double MIN_VEL_Y = 0.0;
    public static final double MAX_VEL_Y = 0.0;
    public static final double MAX_VEL_TH = 1.5;
    public static final double MIN_VEL_TH = 0.1;
    public static final double MIN_VEL_TH_INPLACE = 0.5;
    public static final double ACC_LIM_X = 5.0;
    public static final double ACC_LIM_Y = 5.0;
    public static final double ACC_LIM_TH = 5.2;

    public static final double MAX_VEL_WITH_OBSTACLES = 0.5;

    public static final double FOOTPRINT_RADIUS = 0.17;

    public static final double TIME_TO_HOLO = 0.4;
    public static final double MIN_ERROR_HOLO = 0.02;
    public static final double MAX_ERROR_HOLO = 0.10;

    public static final double EPS = 0.1;


    public static final boolean CONTROLLED = true;
    public static final boolean USE_OBSTACLES = true;

    public static final int TYPE_VO = 0; //HRVO = 0, RVO = 1, VO = 2,
    public static final boolean ORCA = false;//ORCA OR VO, USE CLEAR PATH METHOD
    public static final boolean CONVEX = true; //FOOTPRINT OR RADIUS
    public static final boolean USE_TRUNCATION = true;//TRUNCATE VOS
    public static final double TRUNC_TIME = 5.0;

    public static final double PUBLISH_POSITIONS_FREQUENCY = 10.0; // VISUALIZATION
    public static final double PUBLISH_ME_FREQUENCY = 20.0; //SHARE POSITION
    public static final double CONTROLLER_FREQUENCY = 20.0; //SIMULATION PERIOD

}
