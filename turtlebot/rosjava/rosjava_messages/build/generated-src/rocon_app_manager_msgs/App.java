package rocon_app_manager_msgs;

public interface App extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/App";
  static final java.lang.String _DEFINITION = "# app name\nstring name\n# user-friendly display name\nstring display_name\nstring description\nstring platform\nstring status\n\n# icon for showing the app\nIcon icon\n#  ordered list (by preference) of pairing clients to interact with this robot app.\nPairingClient[] pairing_clients";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDisplayName();
  void setDisplayName(java.lang.String value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  java.lang.String getPlatform();
  void setPlatform(java.lang.String value);
  java.lang.String getStatus();
  void setStatus(java.lang.String value);
  rocon_app_manager_msgs.Icon getIcon();
  void setIcon(rocon_app_manager_msgs.Icon value);
  java.util.List<rocon_app_manager_msgs.PairingClient> getPairingClients();
  void setPairingClients(java.util.List<rocon_app_manager_msgs.PairingClient> value);
}
