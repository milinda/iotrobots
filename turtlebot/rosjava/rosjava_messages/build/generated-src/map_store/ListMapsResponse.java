package map_store;

public interface ListMapsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_store/ListMapsResponse";
  static final java.lang.String _DEFINITION = "MapListEntry[] map_list";
  java.util.List<map_store.MapListEntry> getMapList();
  void setMapList(java.util.List<map_store.MapListEntry> value);
}
