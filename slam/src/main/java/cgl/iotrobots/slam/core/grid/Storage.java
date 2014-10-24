package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.Point;

public interface Storage {
    public int getXSize();

    public int getYSize();

    public int getPatchSize();

    int getPatchMagnitude();

    void resize(int ixmin, int iymin, int ixmax, int iymax);

    Cell cell(int x, int y);

    boolean isAllocated(int x, int y);

    AccessibilityState cellState(int x, int y);

    Point<Integer> patchIndexes(int x, int y);

    Cell cell(Point<Integer> p);

    boolean isAllocated(Point<Integer> p);

    AccessibilityState cellState(Point<Integer> p);

    Point<Integer> patchIndexes(Point<Integer> p);
}
