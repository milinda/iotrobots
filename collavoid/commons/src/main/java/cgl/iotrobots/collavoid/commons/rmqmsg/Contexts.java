package cgl.iotrobots.collavoid.commons.rmqmsg;

import java.util.HashMap;
import java.util.Map;

public class Contexts {
    private Map<String, RMQContext> RMQContexts = new HashMap<String, RMQContext>();

    public Contexts(String sensorName) {
        RMQContexts.put(Constant_msg.KEY_ODOMETRY, new RMQContext(Constant_msg.KEY_ODOMETRY, sensorName));
        RMQContexts.put(Constant_msg.KEY_POSE_ARRAY, new RMQContext(Constant_msg.KEY_POSE_ARRAY, sensorName));
        RMQContexts.put(Constant_msg.KEY_SCAN, new RMQContext(Constant_msg.KEY_SCAN, sensorName));
        RMQContexts.put(Constant_msg.KEY_VELOCITY_CMD, new RMQContext(Constant_msg.KEY_VELOCITY_CMD, sensorName));
        RMQContexts.put(Constant_msg.KEY_BASE_CONFIG, new RMQContext(Constant_msg.KEY_BASE_CONFIG, sensorName));
    }

    public Map<String, RMQContext> getRMQContexts() {
        return RMQContexts;
    }
}
