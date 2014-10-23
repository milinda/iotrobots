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

    public int getPatchSize() {return m_patchMagnitude;}
    public int getPatchMagnitude() {return m_patchMagnitude;}

    public Cell cell(Point<Integer> p) { return cell(p.x, p.y); }
    public boolean isAllocated(Point<Integer> p)  { return isAllocated(p.x,p.y);}
    public AccessibilityState cellState(const IntPoint& p) const { return cellState(p.x,p.y); }
    public Point<Integer> patchIndexes(const IntPoint& p) const { return patchIndexes(p.x,p.y);}

    public void setActiveArea(const PointSet&, bool patchCoords=false);
    PointSet& getActiveArea() const {return m_activeArea; }

    public void assign(HierarchicalArray2D hg) {
        array2D.m_xsize = hg.array2D.m_xsize;
        array2D.m_ysize = hg.array2D.m_ysize;
        array2D.m_cells = new PointAccumulator[array2D.m_xsize][array2D.m_ysize];

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


    AccessibilityState  cellState(int x, int y) const {
        if (this.isInside(patchIndexes(x,y))) {
            if(isAllocated(x,y))
                return (AccessibilityState)((int)Inside|(int)Allocated);
            else
                return Inside;
        }
        return Outside;
    }

    template <class Cell>
    void HierarchicalArray2D<Cell>::allocActiveArea(){
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

    template <class Cell>
    bool HierarchicalArray2D<Cell>::isAllocated(int x, int y) const{
        IntPoint c=patchIndexes(x,y);
        autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
        return (ptr != 0);
    }

    template <class Cell>
    IntPoint HierarchicalArray2D<Cell>::patchIndexes(int x, int y) const{
        if (x>=0 && y>=0)
            return IntPoint(x>>m_patchMagnitude, y>>m_patchMagnitude);
        return IntPoint(-1, -1);
    }

    template <class Cell>
    Cell& HierarchicalArray2D<Cell>::cell(int x, int y){
        IntPoint c=patchIndexes(x,y);
        assert(this->isInside(c.x, c.y));
        if (!this->m_cells[c.x][c.y]){
            Array2D<Cell>* patch=createPatch(IntPoint(x,y));
            this->m_cells[c.x][c.y]=autoptr< Array2D<Cell> >(patch);
            //cerr << "!!! FATAL: your dick is going to fall down" << endl;
        }
        autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
    }

     Cell cell(int x, int y) const{
        assert(isAllocated(x,y));
        IntPoint c=patchIndexes(x,y);
        const autoptr< Array2D<Cell> >& ptr=this->m_cells[c.x][c.y];
        return (*ptr).cell(IntPoint(x-(c.x<<m_patchMagnitude),y-(c.y<<m_patchMagnitude)));
    }
}
