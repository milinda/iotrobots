package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;

public interface IGMap {
    DoublePoint getCenter();

    double getWorldSizeX();

    double getMapResolution();

    double getResolution();

    Size getSize(double xmin, double ymin, double xmax, double ymax);

    Object cell(int x, int y, boolean c);

    Object cell(double x, double y, boolean c);

    boolean isInside(int x, int y);

    boolean isInside(IntPoint p);

    boolean isInsideD(DoublePoint p);

    boolean isInside(double x, double y);

    void resize(double xmin, double ymin, double xmax, double ymax);

    void grow(double xmin, double ymin, double xmax, double ymax);

    IntPoint world2map(double x, double y);

    IntPoint world2map(IntPoint p);

    IntPoint world2map(DoublePoint p);

    IntPoint world2map(DoubleOrientedPoint p);

    DoublePoint map2world(IntPoint p);

    Object cell(IntPoint p, boolean c);

    DoublePoint getM_center();

    double getWorldSizeY();

    double getDelta();

    HierarchicalArray2D getStorage();

    int getMapSizeX();

    int getMapSizeY();

    int getSizeX2();

    int getSizeY2();

    void setM_center(DoublePoint m_center);

    void setWorldSizeX(double worldSizeX);

    void setWorldSizeY(double worldSizeY);

    void setDelta(double delta);

    void setStorage(HierarchicalArray2D storage);

    void setMapSizeX(int mapSizeX);

    void setMapSizeY(int mapSizeY);

    void setSizeX2(int sizeX2);

    void setSizeY2(int sizeY2);
}
