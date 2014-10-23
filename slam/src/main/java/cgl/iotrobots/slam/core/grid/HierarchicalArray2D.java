package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.Point;

import java.util.Comparator;
import java.util.Set;
import java.util.TreeSet;

public class HierarchicalArray2D {
    Array2D array2D;
    Set<Point<Integer>> m_activeArea = new TreeSet<Point<Integer>>(new PointComparator());
    int m_patchMagnitude;
    int m_patchSize;

    public HierarchicalArray2D(int xsize, int ysize, int patchMagnitude) {
        m_patchMagnitude=patchMagnitude;
        m_patchSize=1<<m_patchMagnitude;
    }

    public class PointComparator implements Comparator<Point<Integer>> {
        @Override
        public int compare(Point<Integer> a, Point<Integer> b) {
            boolean equal = a.x<b.x || (a.x==b.x && a.y<b.y);
            return equal ? 1 : 0;
        }
    }

    public HierarchicalArray2D(HierarchicalArray2D hg) {
        assign(hg);
    }

    public void assign(HierarchicalArray2D hg) {
        array2D.m_xsize = hg.array2D.m_xsize;
        array2D.m_ysize = hg.array2D.m_ysize;
        array2D.m_cells = new PointAccumulator[array2D.m_xsize][array2D.m_ysize];

        for (int x = 0; x < array2D.m_xsize; x++) {
            for (int y = 0; y < array2D.m_ysize; y++) {
                array2D.m_cells[x][y] = hg.array2D.m_cells[x][y];
            }
        }
        this.m_activeArea.clear();
        this.m_patchMagnitude = hg.m_patchMagnitude;
        this.m_patchSize = hg.m_patchSize;
    }

    public int getXSize() {
        return array2D.m_xsize;
    }

    public void resize(int xmin, int ymin, int xmax, int ymax) {
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;
        PointAccumulator[][] newcells = new PointAccumulator[xsize][ysize];

        int dx = xmin < 0 ? 0 : xmin;
        int dy = ymin < 0 ? 0 : ymin;
        int Dx = xmax < this.array2D.m_xsize ? xmax : this.array2D.m_xsize;
        int Dy = ymax < this.array2D.m_ysize ? ymax : this.array2D.m_ysize;
        for (int x = dx; x < Dx; x++) {
            for (int y = dy; y < Dy; y++) {
                newcells[x - xmin][y - ymin] = this.array2D.m_cells[x][y];
            }
        }
        this.array2D.m_cells = newcells;
        this.array2D.m_xsize = xsize;
        this.array2D.m_ysize = ysize;
    }

    void setActiveArea(Set<Point<Integer>> aa, boolean patchCoords){
        m_activeArea.clear();
        for (Point<Integer> it : aa) {
            Point<Integer> p;
            if (patchCoords) {
                p = it;
            } else {
                p = patchIndexes(it.x, it.y);
            }
            m_activeArea.add(p);
        }
    }

    public Point<Integer> patchIndexes(int x, int y) {
        if (x>=0 && y>=0) {
            return new Point<Integer>(x >> m_patchMagnitude, y >> m_patchMagnitude);
        }
        return new Point<Integer>(-1, -1);
    }


}
