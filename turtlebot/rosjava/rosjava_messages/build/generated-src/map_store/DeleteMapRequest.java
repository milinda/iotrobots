package map_store;

public interface DeleteMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_store/DeleteMapRequest";
  static final java.lang.String _DEFINITION = "# Service used to delete a given map\n\nstring map_id\n";
  java.lang.String getMapId();
  void setMapId(java.lang.String value);
}
