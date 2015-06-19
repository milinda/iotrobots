package cgl.iotrobots.collavoid.commons.rmqmsg;

public abstract class Constant_msg {

    public static final String KEY_VELOCITY_CMD = "VelocityCMD";
    public static final String KEY_ODOMETRY = "Odometry";
    public static final String KEY_SCAN = "Scan";
    public static final String KEY_POSE_ARRAY = "PoseArray";
    public static final String KEY_POSE_SHARE = "PoseShare";
    public static final String KEY_BASE_CONFIG = "baseConfig";

    public static final String TYPE_EXCHANGE_DIRECT = "direct";
    public static final String TYPE_EXCHANGE_FANOUT = "fanout";
    public static final String TYPE_EXCHANGE_TOPIC = "topic";

    public static final String RMQ_QUEUE_PREFIX = "_Queue_";

}
