package rocon_app_manager_msgs;

public interface StartAppRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/StartAppRequest";
  static final java.lang.String _DEFINITION = "# Name of the app to launch\nstring name\nRemapping[] remappings\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.util.List<rocon_app_manager_msgs.Remapping> getRemappings();
  void setRemappings(java.util.List<rocon_app_manager_msgs.Remapping> value);
}
