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
        public static final String DATA_STREAM = "scans";
        public static final String GATHER_STREAM = "particle";
        public static final String SEND_STREAM = "best_particle";
        public static final String CONTROL_STREAM = "control";
        public static final String TIME_FIELD = "time";
        public static final String LASER_SCAN_FIELD = "laser_scan";
        public static final String PARTICLE_VALUE_FIELD = "particle_value";
        public static final String SENSOR_ID_FIELD = "sensorID";
        public static final String TRACE_FIELD = "trace";
        public static final String ASSIGNMENT_STREAM = "assignments";
    }

    public abstract class Messages {
        public static final String BROADCAST_EXCHANGE = "slam_broadcast";
        public static final String DIRECT_EXCHANGE = "slam_direct";
        public static final String PARTICLE_ASSIGNMENT_ROUTING_KEY = "pa";
        public static final String PARTICLE_MAP_ROUTING_KEY ="pm";
        public static final String PARTICLE_VALUE_ROUTING_KEY ="pv";
        public static final String READY_ROUTING_KEY = "ready";
    }

    public abstract class Topology {
        public static final String RECEIVE_SPOUT = "receive_spout";
        public static final String WORKER_BOLT = "scan_match_bolt";
        public static final String GATHER_BOLT = "gather_bolt";
        public static final String MAP_SEND_BOLD = "map_send_bolt";
        public static final String RESULT_SEND_BOLT = "best_send_bolt";
        public static final String CONTROL_SPOUT = "control_spout";
        public static final String BROADCAST_BOLT = "dispatcher_bolt";
    }
}
