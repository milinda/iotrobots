package map_store;

public interface PublishMapRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_store/PublishMapRequest";
  static final java.lang.String _DEFINITION = "# Service used to publish a given map from the database to the /map topic.\n\nstring map_id\n";
  java.lang.String getMapId();
  void setMapId(java.lang.String value);
}
