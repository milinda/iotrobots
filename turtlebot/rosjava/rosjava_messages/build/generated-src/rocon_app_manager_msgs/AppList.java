package rocon_app_manager_msgs;

public interface AppList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/AppList";
  static final java.lang.String _DEFINITION = "App[] available_apps\nApp[] running_apps\n";
  java.util.List<rocon_app_manager_msgs.App> getAvailableApps();
  void setAvailableApps(java.util.List<rocon_app_manager_msgs.App> value);
  java.util.List<rocon_app_manager_msgs.App> getRunningApps();
  void setRunningApps(java.util.List<rocon_app_manager_msgs.App> value);
}
