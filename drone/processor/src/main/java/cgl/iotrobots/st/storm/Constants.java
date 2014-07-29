package cgl.iotrobots.st.storm;

public abstract class Constants {
    public static final String FRAME_FIELD = "frame";
    public static final String TIME_FIELD = "time";
    public static final String CONTROL_FIELD = "control";

    public static final String FRAME_RECEIVE_SPOUT = "frame_receive";
    public static final String DECODE_BOLT = "decode";
    public static final String TRACKING_BOLT = "tracking";
    public static final String PLANING_BOLT = "planing";
    public static final String SEND_COMMAND_BOLT = "send_command";
    public static final String DECODE_AND_TRACKING_BOLT = "decode_tracking";

    public static final String ARGS_URL = "url";
    public static final String ARGS_NAME = "name";
    public static final String ARGS_LOCAL = "local";
}
