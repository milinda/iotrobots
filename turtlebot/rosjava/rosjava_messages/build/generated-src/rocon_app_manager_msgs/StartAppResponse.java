package rocon_app_manager_msgs;

public interface StartAppResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/StartAppResponse";
  static final java.lang.String _DEFINITION = "# true ifapp started, false otherwise\nbool started\n# classifying start success/failure, see ErrorCodes.msg\nint32 error_code\n# human friendly string for debugging (usually upon error)\nstring message\n# Namespace where the app interface can be found\nstring app_namespace";
  boolean getStarted();
  void setStarted(boolean value);
  int getErrorCode();
  void setErrorCode(int value);
  java.lang.String getMessage();
  void setMessage(java.lang.String value);
  java.lang.String getAppNamespace();
  void setAppNamespace(java.lang.String value);
}
