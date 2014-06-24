package map_store;

public interface RenameMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_store/RenameMapRequest";
  static final java.lang.String _DEFINITION = "# Service used to rename a given map.\n\nstring map_id\nstring new_name\n";
  java.lang.String getMapId();
  void setMapId(java.lang.String value);
  java.lang.String getNewName();
  void setNewName(java.lang.String value);
}
