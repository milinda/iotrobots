package rocon_app_manager_msgs;

public interface PlatformInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "rocon_app_manager_msgs/PlatformInfo";
  static final java.lang.String _DEFINITION = "# Provides platform details from the app manager.\n\n######################## Platform Triple ########################\n\nstring PLATFORM_ANY=*\nstring PLATFORM_LINUX=linux\nstring PLATFORM_WINDOZE=windoze\nstring PLATFORM_ANDROID=android\n\nstring SYSTEM_CUSTOM=custom\nstring SYSTEM_ROS=ros\nstring SYSTEM_OPROS=opros\n\n# Valid robot types, though this is totally not\n# official, and we aren\'t relying on it.\nstring ROBOT_ANY=*\nstring ROBOT_PC=pc\nstring ROBOT_ROBOSEM=robosem\nstring ROBOT_KOBUKI=kobuki\nstring ROBOT_TURTLEBOT=turtlebot\n\n########################### Variables ###########################\n\nstring platform\nstring system\nstring robot\nstring name\nIcon icon";
  static final java.lang.String PLATFORM_ANY = "*";
  static final java.lang.String PLATFORM_LINUX = "linux";
  static final java.lang.String PLATFORM_WINDOZE = "windoze";
  static final java.lang.String PLATFORM_ANDROID = "android";
  static final java.lang.String SYSTEM_CUSTOM = "custom";
  static final java.lang.String SYSTEM_ROS = "ros";
  static final java.lang.String SYSTEM_OPROS = "opros";
  static final java.lang.String ROBOT_ANY = "*";
  static final java.lang.String ROBOT_PC = "pc";
  static final java.lang.String ROBOT_ROBOSEM = "robosem";
  static final java.lang.String ROBOT_KOBUKI = "kobuki";
  static final java.lang.String ROBOT_TURTLEBOT = "turtlebot";
  java.lang.String getPlatform();
  void setPlatform(java.lang.String value);
  java.lang.String getSystem();
  void setSystem(java.lang.String value);
  java.lang.String getRobot();
  void setRobot(java.lang.String value);
  java.lang.String getName();
  void setName(java.lang.String value);
  rocon_app_manager_msgs.Icon getIcon();
  void setIcon(rocon_app_manager_msgs.Icon value);
}
