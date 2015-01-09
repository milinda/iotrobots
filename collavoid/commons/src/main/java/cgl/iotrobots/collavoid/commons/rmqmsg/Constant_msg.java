package cgl.iotrobots.collavoid.commons.rmqmsg;

public abstract class Constant_msg {

    public static final String KEY_VELOCITY_CMD = "VelocityCMD";
    public static final String KEY_ODOMETRY = "Odometry";
    public static final String KEY_SCAN = "Scan";
    public static final String KEY_PARTICLE_CLOUD = "ParticleCloud";
    public static final String KEY_POSE_SHARE = "PoseShare";
    public static final String KEY_START_GOAL = "startGoal";
    public static final String KEY_BASE_CONFIG = "baseConfig";
    public static final String KEY_START = "start";
    public static final String KEY_GOAL = "goal";
    
    public static final String TYPE_EXCHANGE_DIRECT = "direct";
    public static final String TYPE_EXCHANGE_FANOUT = "fanout";
    public static final String TYPE_EXCHANGE_TOPIC = "topic";

    // for test
    public static final String AGENT_ID_PREFIX = "robot";
    public static final String RMQ_EXCHANGE_SUFFIX = "_rmq";
    public static final String RMQ_QUEUE_PREFIX = "_Queue_";
    public static final String RMQ_ROUTINGKEY_PREFIX = "RoutingKey_";
    public static final String RMQ_QUEUE_SUFFIX = "_Spout";

    public static final String RMQ_URL = "amqp://localhost:5672";


}
