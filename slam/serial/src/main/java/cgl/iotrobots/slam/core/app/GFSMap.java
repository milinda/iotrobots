package cgl.iotrobots.slam.core.app;

import cgl.iotrobots.slam.core.utils.IntPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * This is the map output by the MapUpdater
 */
public class GFSMap {
    public int width;
    public int height;

    public double resolution;

    public int data[];

    public List<IntPoint> currentPos = Collections.synchronizedList(new ArrayList<IntPoint>());

    public class Position {
        double x, y, z, w;
    }

    Position origin = new Position();

    Position originOrientation = new Position();

    public GFSMap() {
    }

    public GFSMap(int width, int height) {
        this.width = width;
        this.height = height;

        data = new int[width * height];
    }

    public void resize(int size) {
        data = new int[size];
    }
}
