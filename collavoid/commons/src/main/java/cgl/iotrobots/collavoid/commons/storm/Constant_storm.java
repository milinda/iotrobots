package cgl.iotrobots.collavoid.commons.storm;

import cgl.iotrobots.collavoid.commons.rmqmsg.IotRMQContext;

import java.util.HashMap;
import java.util.Map;

public abstract class Constant_storm {
    public abstract static class FIELDS {
        //general
        public static final String TIME_FIELD = "time";
        public static final String SENSOR_ID_FIELD = "sensorID";
        //global planner
        public static final String BASE_CONFIG_FIELD = "baseConfig";//in streaming

        //local planner
        public static final String VELOCITY_COMMAND_FIELD = "velocityCommand";

        //odometry
        public static final String ODOMETRY_FIELD = "odometry";
        //scan get obstacles
        public static final String SCAN_FIELD = "scan";
        //pose share spout
        public static final String POSE_SHARE_FIELD = "poseShare";
        // get minkowski footprint
        public static final String POSE_ARRAY_FIELD = "pose_array";
        // compute vo lines
        public static final String AGENT_FIELD = "agent";
        public static final String ACC_LINES_FIELD = "accLines";// to joint them need to separate each type
        public static final String NH_LINES_FIELD = "nhLines";
        public static final String OBSTACLE_LINES_FIELD = "obstacleLines";// for orca
        public static final String AGENT_VO_FIELD = "agentVO";
        public static final String OBSTACLE_VO_FIELD = "obstacleVO";
        public static final String SEQUENCE_FIELD = "sequence";

        // timer
        public static final String TIMER_TICK_FIELD = "timerTick";
        //control command
        public static final String COMMAND_FILED = "command";

        //test time delay
        public static final String EMIT_TIME_FIELD = "emitTime";
        public static final String TASK_ID_FIELD = "taskid";


        // simple topology
        public static final String AGENT_STATE_FIELD = "agentState";
        public static final String CTL_PUB_TIME_FIELD = "ctlPubTime";
        public static final String TRUNC_TIME_FIELD = "truncTime";
        public static final String AGENT_IDX_FIELD = "agentIndex";
    }

    public abstract class Streams {
        //local planner
        public static final String VELOCITY_COMMAND_STREAM = "velocityCommand";
        public static final String PUBLISHME_STREAM = "publishMe";
        public static final String CALCULATE_VELOCITY_CMD_STREAM = "calculateVelocity";
        public static final String COMMAND_STREAM = "command";
        public static final String TIMEOUT_STREAM = "timeout";

        public static final String PUBLISHME_PUB_STREAM = "mePublisher";

        // simple topology
        public static final String CTL_PUB_TIME_STREAM = "ctlPubTime";
        public static final String CTLPUB_TIMEER_STREAM = "ctlPubTimer";
        public static final String POSE_SHARE_MSG_STREAM = "poseShareMsg";
        public static final String AGENT_STREAM = "agentStream";
        public static final String ACK_STREAM = "ack";
    }

    public abstract class Components {
        public static final String GLOBAL_PLANNER_COMPONENT = "globalPlanner";
        public static final String ODOMETRY_SPOUT_COMPONENT = "odometry_receiver";
        public static final String SCAN_COMPONENT = "scan_receiver";
        public static final String POSE_SHARE_COMPONENT = "pose_share_out_receiver";
        public static final String POSE_ARRAY_COMPONENT = "pose_array_receiver";
        public static final String COMMAND_SPOUT_COMPONENT = "command_receiver";// in iot
        public static final String BASE_CONFIG_COMPONENT = "base_config_receiver";// in streaming
        public static final String VELOCITY_COMMAND_PUBLISHER_COMPONENT = "vel_cmd_sender";
        public static final String VELOCITY_COMPUTE_COMPONENT = "velocityCompute";
        public static final String AGENT_STATE_COMPONENT = "agentState";
        public static final String TIMER_SPOUT_COMPONENT = "timerSpout";//in streaming
        public static final String POSE_SHARE_PUB_COMPONENT = "pose_share_in_sender";// iotcloud component
        public static final String DISPATCHER_COMPONENT = "dispatcher";

    }

    public abstract class IotCloud {
        public abstract class Channels {
            public static final String ODOMETRY_CHANNEL = "odometry";
            public static final String SCAN_CHANNEL = "scan";
            public static final String POSE_ARRAY_CHANNEL = "pose_array";
            public static final String COMMAND_CHANNEL = "command";
            public static final String POSE_SHARE_PUB_COMPONENT = "pose_share_in";
            public static final String VELOCITY_PUBLISHER_CHANNEL = "vel_cmd";
        }

        public static final String SENSOR_NAME = "collisionAvoid";
        public static final String AGENT_INDEX = "agentIndex";
        public static final String TRANSPORT = "rabbitmq";
        public static final String VELOCITY_QUEUE = "velocityQueue";

        public static final String ROS_IP_KEY = "localIP";
        public static final String ROS_MASTER_URI_KEY = "rosMasterUri";
        public static final String SENSOR_NUMBER_KEY = "numberSensors";
        public static final String SITES_KEY = "sites";
    }

    public static class IotMsgContexts {
        public Map<String, IotRMQContext> Contexts = new HashMap<String, IotRMQContext>();

        public IotMsgContexts() {
            Contexts.put(Components.ODOMETRY_SPOUT_COMPONENT, new IotRMQContext(Components.ODOMETRY_SPOUT_COMPONENT));
            Contexts.put(Components.POSE_ARRAY_COMPONENT, new IotRMQContext(Components.POSE_ARRAY_COMPONENT));
            Contexts.put(Components.SCAN_COMPONENT, new IotRMQContext(Components.SCAN_COMPONENT));
            Contexts.put(Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT,
                    new IotRMQContext(Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT));
            Contexts.put(Components.COMMAND_SPOUT_COMPONENT, new IotRMQContext(Components.COMMAND_SPOUT_COMPONENT));
            Contexts.put(Components.POSE_SHARE_PUB_COMPONENT, new IotRMQContext(Components.POSE_SHARE_PUB_COMPONENT));
            Contexts.put(Components.POSE_SHARE_COMPONENT, new IotRMQContext(Components.POSE_SHARE_COMPONENT));
        }

    }

    public static abstract class Command {
        public static final String RESET_CMD = "reset";
        public static final String DISCONNECTION_CMD = "disconnected";
    }

    public static abstract class Misc {
        public static final long CHECK_ODOMETRY_TIMEOUT_INTERVAL = 5;//in seconds
        public static final long ODOMETRY_TIMEOUT = 5;//inseconds
    }

}
