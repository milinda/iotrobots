package map_store;

public interface ListMaps extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_store/ListMaps";
  static final java.lang.String _DEFINITION = "# Service used to list the most recent map from every map-making session.\n\n# No arguments at this time.\n---\nMapListEntry[] map_list\n";
}
