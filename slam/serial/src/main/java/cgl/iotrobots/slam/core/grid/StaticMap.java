package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;

import java.util.Set;

/**
 * This is a straight forward map.
 */
public class StaticMap implements IGMap {
    public DoublePoint center;
    double worldSizeX, worldSizeY, delta;
    public Object [][]storage;
    public int mapSizeX, mapSizeY;
    public int sizeX2, sizeY2;
    public static final int DEFAULT_PATCH = 5;

    public StaticMap() {
    }

    public StaticMap(int mapSizeX, int mapSizeY, double delta) {
        storage = new Object[mapSizeX][mapSizeY];
        worldSizeX = mapSizeX * delta;
        worldSizeY = mapSizeY * delta;
        this.delta = delta;
        this.mapSizeX = mapSizeX;
        this.mapSizeY = mapSizeY;
        center = new DoublePoint(0.5 * worldSizeX, 0.5 * worldSizeY);
        sizeX2 = this.mapSizeX >> 1;
        sizeY2 = this.mapSizeY >> 1;
    }

    public StaticMap(DoublePoint center, double worldSizeX, double worldSizeY, double delta) {
        int xSize = new Double(Math.ceil(worldSizeX / delta)).intValue();
        int ySize = new Double(Math.ceil(worldSizeY / delta)).intValue();
        storage = new Object[xSize][ySize];
        this.center = center;
        this.worldSizeX = worldSizeX;
        this.worldSizeY = worldSizeY;
        this.delta = delta;
        mapSizeX = xSize;
        mapSizeY = ySize;
        sizeX2 = mapSizeX >> 1;
        sizeY2 = mapSizeY >> 1;
    }

    public StaticMap(DoublePoint center, double xmin, double ymin, double xmax, double ymax, double delta) {
        int xSize = new Double(Math.ceil((xmax - xmin) / delta)).intValue();
        int ySize = new Double(Math.ceil((ymax - ymin) / delta)).intValue();
        storage = new Object[xSize][ySize];
        this.center = center;
        worldSizeX = xmax - xmin;
        worldSizeY = ymax - ymin;
        this.delta = delta;
        mapSizeX = xSize;
        mapSizeY = ySize;
        sizeX2 = (int) Math.round((this.center.x - xmin) / this.delta);
        sizeY2 = (int) Math.round((this.center.y - ymin) / this.delta);
    }

    public DoublePoint getCenter() {
        return center;
    }

    public double getWorldSizeX() {
        return worldSizeX;
    }

    public double getMapResolution() {
        return delta;
    }

    public double getResolution() {
        return delta;
    }

    public Size getSize(double xmin, double ymin, double xmax, double ymax) {
        DoublePoint min = map2world(new IntPoint(0, 0));
        DoublePoint max = map2world(new IntPoint(mapSizeX - 1, mapSizeY - 1));
        return new Size(min.x, min.y, max.x, max.y);
    }

    public Object cell(int x, int y, boolean c) {
        Object pa = storage[x][y];
        if (pa != null) {
            return pa;
        } else {
            return new PointAccumulator();
        }
    }

    public Object cell(double x, double y, boolean c) {
        return cell(new IntPoint(new Double(x).intValue(), new Double(y).intValue()), c);
    }

    public boolean isInside(int x, int y) {
        return x < mapSizeX && y < mapSizeY;
    }

    public boolean isInside(IntPoint p) {
        return isInside(p.x, p.y);
    }

    public boolean isInsideD(DoublePoint p) {
        return isInside(p.x, p.y);
    }

    public boolean isInside(double x, double y) {
        return isInside(world2map(new IntPoint(new Double(x).intValue(), new Double(y).intValue())));
    }

    public void resize(double xmin, double ymin, double xmax, double ymax) {
        IntPoint imin = world2map(xmin, ymin);
        IntPoint imax = world2map(xmax, ymax);
        int pxmin, pymin, pxmax, pymax;
        pxmin = (int) Math.floor((double) imin.x);
        pxmax = (int) Math.ceil((double) imax.x);
        pymin = (int) Math.floor((double) imin.y);
        pymax = (int) Math.ceil((double) imax.y);
        resizeArray(pxmin, pymin, pxmax, pymax);
        mapSizeX = mapSizeX << 1;
        mapSizeY = mapSizeY << 1;
        worldSizeX = xmax - xmin;
        worldSizeY = ymax - ymin;
        sizeX2 -= pxmin * (1 << 1);
        sizeY2 -= pymin * (1 << 1);
    }

    @Override
    public void grow(double xmin, double ymin, double xmax, double ymax) {

    }

    public void resizeArray(int xmin, int ymin, int xmax, int ymax) {
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;
        Object[][] newcells = new Object[xsize][ysize];

        int dx = xmin < 0 ? 0 : xmin;
        int dy = ymin < 0 ? 0 : ymin;
        int Dx = xmax < mapSizeX ? xmax : mapSizeX;
        int Dy = ymax < mapSizeY ? ymax : mapSizeY;
        for (int x = dx; x < Dx; x++) {
            System.arraycopy(storage[x], dy, newcells[x - xmin], dy - ymin, Dy - dy);
        }
        this.storage = newcells;
        this.mapSizeX = xsize;
        this.mapSizeY = ysize;
    }


    public IntPoint world2map(double x, double y) {
        return new IntPoint((int) Math.round((x - center.x) / delta) + sizeX2,
                (int) Math.round((y - center.y) / delta) + sizeY2);
    }

    public IntPoint world2map(IntPoint p) {
        return new IntPoint((int) Math.round((p.x - center.x) / delta) + sizeX2,
                (int) Math.round((p.y - center.y) / delta) + sizeY2);
    }

    public IntPoint world2map(DoublePoint p) {
        return new IntPoint((int) Math.round((p.x - center.x) / delta) + sizeX2,
                (int) Math.round((p.y - center.y) / delta) + sizeY2);
    }

    public IntPoint world2map(DoubleOrientedPoint p) {
        return new IntPoint((int) Math.round((p.x - center.x) / delta) + sizeX2,
                (int) Math.round((p.y - center.y) / delta) + sizeY2);
    }

    public DoublePoint map2world(IntPoint p) {
        return new DoublePoint((p.x - sizeX2) * delta + center.x,
                (p.y - sizeY2) * delta + center.y);
    }

    public Object cell(IntPoint p, boolean c) {
        PointAccumulator pa = (PointAccumulator) cell(p.x, p.y, c);
        if (pa != null) {
            return pa;
        } else {
            return new PointAccumulator();
        }
    }

    public DoublePoint getM_center() {
        return center;
    }

    public double getWorldSizeY() {
        return worldSizeY;
    }

    public double getDelta() {
        return delta;
    }

    public HierarchicalArray2D getStorage() {
        return null;
    }

    public int getMapSizeX() {
        return mapSizeX;
    }

    public int getMapSizeY() {
        return mapSizeY;
    }

    public int getSizeX2() {
        return sizeX2;
    }

    public int getSizeY2() {
        return sizeY2;
    }

    public void setM_center(DoublePoint m_center) {
        this.center = m_center;
    }

    public void setWorldSizeX(double worldSizeX) {
        this.worldSizeX = worldSizeX;
    }

    public void setWorldSizeY(double worldSizeY) {
        this.worldSizeY = worldSizeY;
    }

    public void setDelta(double delta) {
        this.delta = delta;
    }

    @Override
    public void setStorage(HierarchicalArray2D storage) {

    }

    public void setStorage(Object [][]storage) {
        this.storage = storage;
    }

    public void setMapSizeX(int mapSizeX) {
        this.mapSizeX = mapSizeX;
    }

    public void setMapSizeY(int mapSizeY) {
        this.mapSizeY = mapSizeY;
    }

    public void setSizeX2(int sizeX2) {
        this.sizeX2 = sizeX2;
    }

    public void setSizeY2(int sizeY2) {
        this.sizeY2 = sizeY2;
    }

    @Override
    public void setActiveArea(Set<IntPoint> aa, boolean patchCoords) {

    }

    @Override
    public void allocActiveArea() {

    }


}
