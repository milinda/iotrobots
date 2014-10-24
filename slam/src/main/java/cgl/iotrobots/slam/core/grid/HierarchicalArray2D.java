package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.Point;

import java.util.Comparator;
import java.util.Set;
import java.util.TreeSet;

public class HierarchicalArray2D implements Storage {
    Array2D array2D;
    Set<Point<Integer>> m_activeArea = new TreeSet<Point<Integer>>(new PointComparator());
    int m_patchMagnitude;
    int m_patchSize;

    public HierarchicalArray2D(int xsize, int ysize, int patchMagnitude) {
        m_patchMagnitude = patchMagnitude;
        m_patchSize = 1 << m_patchMagnitude;
    }

    public class PointComparator implements Comparator<Point<Integer>> {
        @Override
        public int compare(Point<Integer> a, Point<Integer> b) {
            boolean equal = a.x < b.x || (a.x.equals(b.x) && a.y < b.y);
            return equal ? 1 : 0;
        }
    }

    public HierarchicalArray2D(HierarchicalArray2D hg) {
        assign(hg);
    }

    public int getPatchSize() {
        return m_patchMagnitude;
    }
    public int getPatchMagnitude() {
        return m_patchMagnitude;
    }

    public Cell cell(Point<Integer> p) {
        return cell(p.x, p.y);
    }

    public boolean isAllocated(Point<Integer> p)  {
        return isAllocated(p.x,p.y);
    }

    public AccessibilityState cellState(Point<Integer> p) {
        return cellState(p.x,p.y);
    }

    public Point<Integer> patchIndexes(Point<Integer> p) {
        return patchIndexes(p.x,p.y);
    }

    Set<Point<Integer>> getActiveArea() {
        return m_activeArea;
    }

    public void assign(HierarchicalArray2D hg) {
        array2D.m_xsize = hg.array2D.m_xsize;
        array2D.m_ysize = hg.array2D.m_ysize;
        array2D.m_cells = new Cell[array2D.m_xsize][array2D.m_ysize];

        for (int x = 0; x < array2D.m_xsize; x++) {
            System.arraycopy(hg.array2D.m_cells[x], 0, array2D.m_cells[x], 0, array2D.m_ysize);
        }
        this.m_activeArea.clear();
        this.m_patchMagnitude = hg.m_patchMagnitude;
        this.m_patchSize = hg.m_patchSize;
    }

    public int getXSize() {
        return array2D.m_xsize;
    }

    public int getYSize() {
        return array2D.m_ysize;
    }

    public void resize(int xmin, int ymin, int xmax, int ymax) {
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;
        Cell[][] newcells = new Cell[xsize][ysize];

        int dx = xmin < 0 ? 0 : xmin;
        int dy = ymin < 0 ? 0 : ymin;
        int Dx = xmax < this.array2D.m_xsize ? xmax : this.array2D.m_xsize;
        int Dy = ymax < this.array2D.m_ysize ? ymax : this.array2D.m_ysize;
        for (int x = dx; x < Dx; x++) {
            System.arraycopy(this.array2D.m_cells[x], dy, newcells[x - xmin], dy - ymin, Dy - dy);
        }
        this.array2D.m_cells = newcells;
        this.array2D.m_xsize = xsize;
        this.array2D.m_ysize = ysize;
    }

    void setActiveArea(Set<Point<Integer>> aa, boolean patchCoords) {
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
        if (x >= 0 && y >= 0) {
            return new Point<Integer>(x >> m_patchMagnitude, y >> m_patchMagnitude);
        }
        return new Point<Integer>(-1, -1);
    }

    Array2D createPatch(Point<Integer> p) {
        return new Array2D(1<<m_patchMagnitude, 1<<m_patchMagnitude);
    }


    public AccessibilityState  cellState(int x, int y) {
        if (array2D.isInside(patchIndexes(new Point<Integer>(x, y)))) {
            if (isAllocated(x,y)) {
                return AccessibilityState.Inside | AccessibilityState.Allocated;
            }
            else {
                return AccessibilityState.Inside;
            }
        }
        return AccessibilityState.Outside;
    }

    void allocActiveArea(){
        for (PointSet::const_iterator it= m_activeArea.begin(); it!=m_activeArea.end(); ++it){
            const autoptr< Array2D<Cell> >& ptr=this->m_cells[it->x][it->y];
            Array2D<Cell>* patch=0;
            if (!ptr){
                patch=createPatch(*it);
            } else{
                patch=new Array2D<Cell>(*ptr);
            }
            this->m_cells[it->x][it->y]=autoptr< Array2D<Cell> >(patch);
        }
    }

    public boolean isAllocated(int x, int y) {
        Point<Integer> c=patchIndexes(x,y);
        Cell val = array2D.m_cells[c.x][c.y];
        return (val != null);
    }

    public Cell cell(int x, int y){
        Point<Integer> c = patchIndexes(x,y);
        assert(array2D.isInside(c.x, c.y));
        if (array2D.m_cells[c.x][c.y] != null){
            Array2D patch= createPatch(new Point<Integer>(x,y));
            array2D.m_cells[c.x][c.y] = patch;
        }
        autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
    }
}
