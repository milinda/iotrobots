package edu.iu.cs.storm.collectives.app;

public class Constants {
    public static final String ARGS_NAME = "name";
    public static final String ARGS_LOCAL = "local";
    public static final String ARGS_DS_MODE = "ds_mode";
    public static final String ARGS_PARALLEL = "p";
    public static final String ARGS_IOT_CLOUD = "i";
    public static final String ARGS_PARTICLES = "pt";

    // configurations
    public static final String RABBITMQ_URL = "rabbitmq_url";

    public abstract class Fields {
        public static final String BODY = "body";
        public static final String DATA_STREAM = "data";
        public static final String BROADCAST_STREAM = "broadcast";
        public static final String GATHER_STREAM = "gather";
        public static final String SEND_STREAM = "send";
        public static final String READY_STREAM = "ready";
        public static final String CONTROL_STREAM = "control";
        public static final String TIME_FIELD = "time";
        public static final String DATA_FIELD = "data";
        public static final String SENSOR_ID_FIELD = "sensorID";
        public static final String TRACE_FIELD = "trace";
    }

    public abstract class Topology {
        public static final String RECEIVE_SPOUT = "receive_spout";
        public static final String WORKER_BOLT = "worker_bolt";
        public static final String GATHER_BOLT = "gather_bolt";
        public static final String RESULT_SEND_BOLT = "send_bolt";
        public static final String CONTROL_SPOUT = "control_spout";
        public static final String BROADCAST_BOLT = "broadcast_bolt";
    }
}
