package cgl.iotrobots.collavoid.commons.planners;

public class Parameters {
    public static String BASE_FRAME_SUFFIX = "_base";
    public static String GLOBAL_FRAME = "map";

    // local planner parameters
    public static double ROT_STOPPED_VELOCITY = 0.02;
    public static double TRANS_STOPPED_VELOCITY = 0.01;
    public static double YAW_GOAL_TOLERANCE = 0.10;
    public static double XY_GOAL_TOLERANCE = 0.10;
    public static boolean LATCH_XY_GOAL_TOLERANCE = true;
    public static boolean IGNORE_GOAL_YAW = false;

    // agent parameters
    public static boolean HOLO_ROBOT = false;
    public static double WHEEL_BASE = 0.25;

    public static double MAX_VEL_X = 0.5;
    public static double MIN_VEL_X = 0.05;
    public static double MIN_VEL_Y = 0.0;
    public static double MAX_VEL_Y = 0.0;
    public static double MAX_VEL_TH = 1.5;
    public static double MIN_VEL_TH = 0.1;
    public static double MIN_VEL_TH_INPLACE = 0.5;
    public static double ACC_LIM_X = 5.0;
    public static double ACC_LIM_Y = 5.0;
    public static double ACC_LIM_TH = 5.2;

    public static double MAX_VEL_WITH_OBSTACLES = 0.5;

    public static double FOOTPRINT_RADIUS = 0.17;

    public static double TIME_TO_HOLO = 0.4;
    public static double MIN_ERROR_HOLO = 0.02;
    public static double MAX_ERROR_HOLO = 0.10;

    public static double EPS = 0.1;


    public static boolean CONTROLLED = true;
    public static boolean USE_OBSTACLES = true;

    public static int TYPE_VO = 0; //HRVO = 0, RVO = 1, VO = 2,
    public static boolean ORCA = false;//ORCA OR VO, USE CLEAR PATH METHOD
    public static boolean CONVEX = true; //FOOTPRINT OR RADIUS
    public static boolean USE_TRUNCATION = true;//TRUNCATE VOS
    public static double TRUNC_TIME = 5.0;

    public static double PUBLISH_POSITIONS_FREQUENCY = 10.0; // VISUALIZATION
    public static double PUBLISH_ME_FREQUENCY = 20.0; //SHARE POSITION
    public static double CONTROLLER_FREQUENCY = 20.0; //SIMULATION PERIOD

    // allowed error for position and heading
    public static double EPSILON = 0.001;

}
