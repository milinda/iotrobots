package cgl.iotrobots.slam.core.sample;

import java.security.PublicKey;

public class OutMap {
    public int width;
    public int height;

    public double resolution;

    public int data[];

    public class Position {
        double x, y, z, w;
    }

    Position origin = new Position();

    Position originOrientation = new Position();

    public OutMap() {
    }

    public OutMap(int width, int height) {
        this.width = width;
        this.height = height;

        data = new int[width * height];
    }

    public void resize(int size) {
        data = new int[size];
    }
}
