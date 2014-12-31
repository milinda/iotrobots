package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.DoublePoint;

public class MapFactory {
    public static IGMap create(DoublePoint center, double worldSizeX, double worldSizeY, double delta) {
        return new GMap(center, worldSizeX, worldSizeY, delta);
    }

    public static IGMap create(DoublePoint center, double xmin, double ymin, double xmax, double ymax, double delta) {
        return new GMap(center, xmin, xmax, ymin, ymax, delta);
    }
}
