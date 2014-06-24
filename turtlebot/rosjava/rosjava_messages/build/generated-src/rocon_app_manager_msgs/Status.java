package rocon_app_manager_msgs;

public interface Status extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/Status";
  static final java.lang.String _DEFINITION = "---\n# Namespace under which applications will run if connected\nstring application_namespace\n\n# Who is controlling the app manager (i.e. who invited it to send it\'s\n# control handles). If the empty string, it is not being controlled \n# and subsequently is available\nstring remote_controller\n\n# Current app status, stopped or running\nstring application_status\n\n# Current app details (if running), a default App() (filled with empty strings and lists) otherwise\nrocon_app_manager_msgs/App application\n";
}
