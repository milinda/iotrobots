package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.Point;

public class GMap <Cell, Storage> {
    Point m_center;
    double m_worldSizeX, m_worldSizeY, m_delta;
    Storage m_storage;
    int m_mapSizeX, m_mapSizeY;
    int m_sizeX2, m_sizeY2;

    public GMap(int mapSizeX, int mapSizeY, double delta) {

    }

    public GMap(Point center, double worldSizeX, double worldSizeY, double delta) {

    }

    public GMap(Point center, double xmin, double ymin, double xmax, double ymax, double delta) {

    }


}
