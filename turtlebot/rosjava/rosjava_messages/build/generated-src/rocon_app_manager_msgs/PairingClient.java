package rocon_app_manager_msgs;

public interface PairingClient extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/PairingClient";
  static final java.lang.String _DEFINITION = "# like \"android\" or \"web\" or \"linux\"\nstring client_type\n\n# like \"intent = ros.android.teleop\" and \"accelerometer = true\", used to choose which ClientApp to use\nKeyValue[] manager_data\n\n# parameters which just get passed through to the client app.\nKeyValue[] app_data";
  java.lang.String getClientType();
  void setClientType(java.lang.String value);
  java.util.List<rocon_app_manager_msgs.KeyValue> getManagerData();
  void setManagerData(java.util.List<rocon_app_manager_msgs.KeyValue> value);
  java.util.List<rocon_app_manager_msgs.KeyValue> getAppData();
  void setAppData(java.util.List<rocon_app_manager_msgs.KeyValue> value);
}
