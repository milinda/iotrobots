package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;

public class GMap {
    public DoublePoint m_center;
    double m_worldSizeX, m_worldSizeY, m_delta;
    public HierarchicalArray2D m_storage;
    public int m_mapSizeX, m_mapSizeY;
    public int m_sizeX2, m_sizeY2;
    public static final int DEFAULT_PATCH = 5;

    public GMap(int mapSizeX, int mapSizeY, double delta) {
        m_storage = new HierarchicalArray2D(mapSizeX, mapSizeY, DEFAULT_PATCH);
        m_worldSizeX = mapSizeX * delta;
        m_worldSizeY = mapSizeY * delta;
        m_delta = delta;
        m_center = new DoublePoint(0.5 * m_worldSizeX, 0.5 * m_worldSizeY);
        m_sizeX2 = m_mapSizeX >> 1;
        m_sizeY2 = m_mapSizeY >> 1;
    }

    public GMap(DoublePoint center, double worldSizeX, double worldSizeY, double delta) {
        m_storage = new HierarchicalArray2D(new Double(Math.ceil(worldSizeX / delta)).intValue(), new Double(Math.ceil(worldSizeY / delta)).intValue(), DEFAULT_PATCH);
        m_center = center;
        m_worldSizeX = worldSizeX;
        m_worldSizeY = worldSizeY;
        m_delta = delta;
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = m_mapSizeX >> 1;
        m_sizeY2 = m_mapSizeY >> 1;
    }

    public GMap(DoublePoint center, double xmin, double ymin, double xmax, double ymax, double delta) {
        m_storage = new HierarchicalArray2D(new Double(Math.ceil((xmax - xmin) / delta)).intValue(), new Double(Math.ceil((ymax - ymin) / delta)).intValue(), DEFAULT_PATCH);
        m_center = center;
        m_worldSizeX = xmax - xmin;
        m_worldSizeY = ymax - ymin;
        m_delta = delta;
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = (int) Math.round((m_center.x - xmin) / m_delta);
        m_sizeY2 = (int) Math.round((m_center.y - ymin) / m_delta);
    }

    public DoublePoint getCenter() {
        return m_center;
    }

    public double getWorldSizeX() {
        return m_worldSizeX;
    }

    public double getWorldSizeY() {
        return m_worldSizeY;
    }

    public int getMapSizeX() {
        return m_mapSizeX;
    }

    public int getMapSizeY() {
        return m_mapSizeY;
    }

    public double getDelta() {
        return m_delta;
    }

    public double getMapResolution() {
        return m_delta;
    }

    public double getResolution() {
        return m_delta;
    }

    public HierarchicalArray2D getStorage() {
        return m_storage;
    }

    public Size getSize(double xmin, double ymin, double xmax, double ymax) {
        DoublePoint min = map2world(new IntPoint(0, 0)), max = map2world(new IntPoint(m_mapSizeX - 1, m_mapSizeY - 1));
        return new Size(min.x, min.y, max.x, max.y);
    }

    public Object cell(int x, int y, boolean c) {
        return cell(new IntPoint(x, y), c);
    }

    public Object cell(double x, double y, boolean c) {
        return cell(new IntPoint(new Double(x).intValue(), new Double(y).intValue()), c);
    }

    public boolean isInside(int x, int y) {
        return (m_storage.cellState(new IntPoint(x, y)) & AccessibilityState.Inside.getVal()) > 0;
    }

    public boolean isInside(IntPoint p) {
        return (m_storage.cellState(p) & AccessibilityState.Inside.getVal()) > 0;
    }

    public boolean isInsideD(DoublePoint p) {
        return isInside(p.x, p.y);
    }

    public boolean isInside(double x, double y) {
        return m_storage.cellState(
                world2map(new IntPoint(new Double(x).intValue(), new Double(y).intValue())))
                == AccessibilityState.Inside.getVal();
    }


    public void resize(double xmin, double ymin, double xmax, double ymax) {
        IntPoint imin = world2map(xmin, ymin);
        IntPoint imax = world2map(xmax, ymax);
        int pxmin, pymin, pxmax, pymax;
        pxmin = (int) Math.floor((double) imin.x / (1 << m_storage.getPatchMagnitude()));
        pxmax = (int) Math.ceil((double) imax.x / (1 << m_storage.getPatchMagnitude()));
        pymin = (int) Math.floor((double) imin.y / (1 << m_storage.getPatchMagnitude()));
        pymax = (int) Math.ceil((double) imax.y / (1 << m_storage.getPatchMagnitude()));
        m_storage.resize(pxmin, pymin, pxmax, pymax);
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_worldSizeX = xmax - xmin;
        m_worldSizeY = ymax - ymin;
        m_sizeX2 -= pxmin * (1 << m_storage.getPatchMagnitude());
        m_sizeY2 -= pymin * (1 << m_storage.getPatchMagnitude());
    }

    public void grow(double xmin, double ymin, double xmax, double ymax) {
        IntPoint imin = world2map(xmin, ymin);
        IntPoint imax = world2map(xmax, ymax);
        if (isInside(imin) && isInside(imax))
            return;
        imin = IntPoint.min(imin, new IntPoint(0, 0));
        imax = IntPoint.max(imax, new IntPoint(m_mapSizeX - 1, m_mapSizeY - 1));
        int pxmin, pymin, pxmax, pymax;
        pxmin = (int) Math.floor((double) imin.x / (1 << m_storage.getPatchMagnitude()));
        pxmax = (int) Math.ceil((double) imax.x / (1 << m_storage.getPatchMagnitude()));
        pymin = (int) Math.floor((double) imin.y / (1 << m_storage.getPatchMagnitude()));
        pymax = (int) Math.ceil((double) imax.y / (1 << m_storage.getPatchMagnitude()));
        m_storage.resize(pxmin, pymin, pxmax, pymax);
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_worldSizeX = xmax - xmin;
        m_worldSizeY = ymax - ymin;
        m_sizeX2 -= pxmin * (1 << m_storage.getPatchMagnitude());
        m_sizeY2 -= pymin * (1 << m_storage.getPatchMagnitude());
    }

    public IntPoint world2map(double x, double y) {
        return new IntPoint((int) Math.round((x - m_center.x) / m_delta) + m_sizeX2, (int) Math.round((y - m_center.y) / m_delta) + m_sizeY2);
    }

    public IntPoint world2map(IntPoint p) {
        return new IntPoint((int) Math.round(((Integer) p.x - m_center.x) / m_delta) + m_sizeX2, (int) Math.round(((Integer) p.y - m_center.y) / m_delta) + m_sizeY2);
    }

    public IntPoint world2map(DoublePoint p) {
        return new IntPoint((int) Math.round(((Double) p.x - m_center.x) / m_delta) + m_sizeX2, (int) Math.round(((Double) p.y - m_center.y) / m_delta) + m_sizeY2);
    }

    public IntPoint world2map(DoubleOrientedPoint p) {
        return new IntPoint((int) Math.round(((Double) p.x - m_center.x) / m_delta) + m_sizeX2, (int) Math.round(((Double) p.y - m_center.y) / m_delta) + m_sizeY2);
    }

    public DoublePoint map2world(IntPoint p) {
        return new DoublePoint((p.x - m_sizeX2) * m_delta + m_center.x,
                (p.y - m_sizeY2) * m_delta + m_center.y);
    }

    public Object cell(IntPoint p, boolean c) {
        int s = m_storage.cellState(p);
        if (c) {
            if ((s & 0x2) > 0)
                return m_storage.cell(p);
        } else {
            if ((s & 0x1) == 0)
                assert false;
            return m_storage.cell(p);
        }

        return new PointAccumulator();
    }

    public Object cell(DoublePoint p, boolean c) {
        IntPoint ip = world2map(p);
        int s = m_storage.cellState(ip);
        //if (! s&Inside) assert(0);
        if (c) {
            if ((s & AccessibilityState.Allocated.getVal()) > 0)
                return m_storage.cell(ip);
        } else {
            if ((s & AccessibilityState.Inside.getVal()) == 0)
                assert false;
            return m_storage.cell(ip);
        }
        return new PointAccumulator();
    }
}
