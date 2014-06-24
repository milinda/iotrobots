package rocon_app_manager_msgs;

public interface StartApp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/StartApp";
  static final java.lang.String _DEFINITION = "# Name of the app to launch\nstring name\nRemapping[] remappings\n---\n# true ifapp started, false otherwise\nbool started\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n# human friendly string for debugging (usually upon error)\nstring message\n# Namespace where the app interface can be found\nstring app_namespace\n";
}
