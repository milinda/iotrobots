package cgl.iotrobots.slam.streaming;

import cgl.iotrobots.slam.core.GFSConfiguration;

import java.util.Map;

/**
 * A utility class for building the GFSConfiguration from the map
 */
public class ConfigurationBuilder {
    public static GFSConfiguration build(Map conf) {
        GFSConfiguration configuration = new GFSConfiguration();

        configuration.set
    }

    public static int getIntProperty(Map conf, String property) {
        return (int) conf.get(property);
    }

    public static double getDoubleProperty(Map conf, String property) {
        return (double) conf.get(property);
    }

    public static String getStringProperty(Map conf, String property) {
        return (String) conf.get(property);
    }
}
