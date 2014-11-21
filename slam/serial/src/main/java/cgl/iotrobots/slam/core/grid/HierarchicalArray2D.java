package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.scanmatcher.PointAccumulator;
import cgl.iotrobots.slam.core.utils.DoublePoint;
import cgl.iotrobots.slam.core.utils.IntPoint;

import java.util.HashSet;
import java.util.Set;

public class HierarchicalArray2D {
    Array2D array2D;
//    Set<Point<Integer>> m_activeArea = new TreeSet<Point<Integer>>(new PointComparator());
    Set<IntPoint> m_activeArea = new HashSet<IntPoint>();
    int m_patchMagnitude = 0;
    int m_patchSize;

    public HierarchicalArray2D(int xsize, int ysize, int patchMagnitude) {
        array2D = new Array2D((xsize>>patchMagnitude), (ysize>>patchMagnitude));

//        for (int i = 0; i < array2D.m_xsize; i++) {
//            for (int j = 0; j < array2D.m_ysize; j++) {
//                Array2D a2d = new Array2D(array2D.m_xsize, array2D.m_ysize);

//                for (int k = 0; k < array2D.m_xsize; k++) {
//                    for (int l = 0; l < array2D.m_ysize; l++) {
//                        a2d.m_cells[k][l] = new PointAccumulator();
//                    }
//                }
//                array2D.m_cells[i][j] = a2d;
//            }
//        }

        m_patchMagnitude = patchMagnitude;
        m_patchSize = 1 << m_patchMagnitude;
    }

//    public class PointComparator implements Comparator<Point<Integer>> {
//        @Override
//        public int compare(Point<Integer> a, Point<Integer> b) {
//            boolean equal = a.x < b.x || (a.x.equals(b.x) && a.y < b.y);
//            return equal ? 1 : 0;
//        }
//    }

    public HierarchicalArray2D(HierarchicalArray2D hg) {
        array2D = new Array2D(hg.array2D.m_xsize>>hg.m_patchMagnitude, hg.array2D.m_ysize>>hg.m_patchMagnitude);
        assign(hg);
    }

    public int getPatchSize() {
        return m_patchMagnitude;
    }

    public int getPatchMagnitude() {
        return m_patchMagnitude;
    }

    public Object cell(IntPoint p) {
        return cell(p.x, p.y);
    }

    public int cellState(IntPoint p) {
        return cellState(p.x,p.y);
    }

    public IntPoint patchIndexes(IntPoint p) {
        return patchIndexes(p.x,p.y);
    }

    Set<IntPoint> getActiveArea() {
        return m_activeArea;
    }

    public void assign(HierarchicalArray2D hg) {
        array2D.m_xsize = hg.array2D.m_xsize;
        array2D.m_ysize = hg.array2D.m_ysize;
        array2D.m_cells = new Object[array2D.m_xsize][array2D.m_ysize];

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
        Object[][] newcells = new Object[xsize][ysize];

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

    public void setActiveArea(Set<IntPoint> aa, boolean patchCoords) {
        m_activeArea.clear();
        for (IntPoint it : aa) {
            IntPoint p;
            if (patchCoords) {
                p = it;
            } else {
                p = patchIndexes(it.x, it.y);
            }
            m_activeArea.add(p);
        }
        // System.out.println("Active area contain: " + m_activeArea.size());
    }

    public IntPoint patchIndexes(int x, int y) {
        if (x >= 0 && y >= 0) {
            return new IntPoint(x >> m_patchMagnitude, y >> m_patchMagnitude);
        }
        return new IntPoint(-1, -1);
    }

    public Array2D createPatch(IntPoint p) {
        return new Array2D(1<<m_patchMagnitude, 1<<m_patchMagnitude);
    }


    public int cellState(int x, int y) {
        IntPoint p = patchIndexes(new IntPoint(x, y));
        if (array2D.isInside(p)) {
            if (isAllocated(p)) {
                return 0x1 | 0x2;
            }
            else {
                return 0x1;
            }
        }
        return 0x0;
    }

    public void allocActiveArea(){
        for (IntPoint it : m_activeArea) {
            Array2D ptr= (Array2D) this.array2D.m_cells[it.x][it.y];
            Array2D patch = null;
            if (ptr == null){
                patch = createPatch(it);
                for (int k = 0; k < patch.m_xsize; k++) {
                    for (int l = 0; l < patch.m_ysize; l++) {
                        patch.m_cells[k][l] = new PointAccumulator();
                    }
                }
            } else{
                patch = createPatch(it);
                for (int k = 0; k < patch.m_xsize; k++) {
                    for (int l = 0; l < patch.m_ysize; l++) {
                        patch.m_cells[k][l] = new PointAccumulator((PointAccumulator) ptr.m_cells[k][l]);
                    }
                }
            }
            this.array2D.m_cells[it.x][it.y] = patch;
        }
    }

    public boolean isAllocated(int x, int y) {
        IntPoint c=patchIndexes(x,y);
        Object val = array2D.m_cells[c.x][c.y];
        return (val != null);
    }

    public boolean isAllocated(IntPoint patchIndexes) {
        IntPoint c=patchIndexes;
        Object val = array2D.m_cells[c.x][c.y];
        return (val != null);
    }

    public Object cell(int x, int y){
        IntPoint c = patchIndexes(x,y);
        assert(array2D.isInside(c.x, c.y));
        if (array2D.m_cells[c.x][c.y] == null){
            Array2D patch= createPatch(new IntPoint(x,y));
            for (int k = 0; k < patch.m_xsize; k++) {
                for (int l = 0; l < patch.m_ysize; l++) {
                    patch.m_cells[k][l] = new PointAccumulator();
                }
            }
            array2D.m_cells[c.x][c.y] = patch;
        }
        Array2D ptr = (Array2D) this.array2D.m_cells[c.x][c.y];
        return ptr.cell(new IntPoint(x-(c.x << m_patchMagnitude), y-(c.y<<m_patchMagnitude)));
    }
}
