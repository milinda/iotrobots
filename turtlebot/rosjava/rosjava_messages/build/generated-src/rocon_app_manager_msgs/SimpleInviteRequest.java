package rocon_app_manager_msgs;

public interface SimpleInviteRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/SimpleInviteRequest";
  static final java.lang.String _DEFINITION = "# Simple version of the Invite service. This is used in pairing mode where\n# the service callback is able to retrieve the gateway name automatically and\n# relay the invitation along the usual invitation service.\n#\n# i.e. much easier for a user to call this to trigger an invite than to provide\n# the full service details\n# \nbool cancel\n";
  boolean getCancel();
  void setCancel(boolean value);
}
