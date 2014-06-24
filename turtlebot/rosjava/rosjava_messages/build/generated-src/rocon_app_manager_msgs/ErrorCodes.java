package rocon_app_manager_msgs;

public interface ErrorCodes extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/ErrorCodes";
  static final java.lang.String _DEFINITION = "# Error types for the rocon app manager\n\n# General\nint8 SUCCESS = 0\n\n# Start\nint8 MULTI_RAPP_NOT_SUPPORTED = 10\n\n# Stop App\nint8 RAPP_IS_NOT_RUNNING = 20\n";
  static final byte SUCCESS = 0;
  static final byte MULTI_RAPP_NOT_SUPPORTED = 10;
  static final byte RAPP_IS_NOT_RUNNING = 20;
}
