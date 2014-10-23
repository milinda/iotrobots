package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.Point;

public class GMap {
    Point<Double> m_center;
    double m_worldSizeX, m_worldSizeY, m_delta;
    HierarchicalArray2D m_storage;
    int m_mapSizeX, m_mapSizeY;
    int m_sizeX2, m_sizeY2;

    public GMap(int mapSizeX, int mapSizeY, double delta) {
        m_worldSizeX=mapSizeX * delta;
        m_worldSizeY=mapSizeY * delta;
        m_delta=delta;
        m_center= new Point<Double>(0.5*m_worldSizeX, 0.5*m_worldSizeY);
        m_sizeX2=m_mapSizeX>>1;
        m_sizeY2=m_mapSizeY>>1;
    }

    public GMap(Point<Double> center, double worldSizeX, double worldSizeY, double delta) {
        m_center = center;
        m_worldSizeX = xmax - xmin;
        m_worldSizeY = ymax - ymin;
        m_delta = delta;
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = (int) Math.round((m_center.x - xmin) / m_delta);
        m_sizeY2 = (int) Math.round((m_center.y - ymin) / m_delta);
    }

    public GMap(Point<Double> center, double xmin, double ymin, double xmax, double ymax, double delta) {
        m_center = center;
        m_worldSizeX = worldSizeX;
        m_worldSizeY = worldSizeY;
        m_delta = delta;
        m_mapSizeX = m_storage.getXSize() << m_storage.getPatchSize();
        m_mapSizeY = m_storage.getYSize() << m_storage.getPatchSize();
        m_sizeX2 = m_mapSizeX >> 1;
        m_sizeY2 = m_mapSizeY >> 1;
    }

    Point getCenter() {return m_center;}
    double getWorldSizeX() {return m_worldSizeX;}
    double getWorldSizeY() {return m_worldSizeY;}
    int getMapSizeX() {return m_mapSizeX;}
    int getMapSizeY() {return m_mapSizeY;}
    double getDelta() { return m_delta;}
    double getMapResolution() { return m_delta;}
    double getResolution() { return m_delta;}

    void getSize(double & xmin, double& ymin, double& xmax, double& ymax) const {
        Point min=map2world(0,0), max=map2world(IntPoint(m_mapSizeX-1, m_mapSizeY-1));
        xmin=min.x, ymin=min.y,  xmax=max.x, ymax=max.y;
    }

    Cell cell(int x, int y) {
        return cell(IntPoint(x, y));
    }


    Cell cell(int x, int y) const  {
        return cell(IntPoint(x, y));
    }

    Cell cell(const IntPoint& p) const;

    Cell cell(double x, double y) {
        return cell(Point(x, y));
    }


    boolean Cell& cell(double x, double y) const {
        return cell(Point(x, y));
    }

    boolean isInside(int x, int y) const {
        return m_storage.cellState(IntPoint(x,y))&Inside;
    }
    boolean isInside(const IntPoint& p) const {
        return m_storage.cellState(p)&Inside;
    }

    boolean isInside(double x, double y) const {
        return m_storage.cellState(world2map(x,y))&Inside;
    }
    boolean isInside(const Point& p) const {
        return m_storage.cellState(world2map(p))&Inside;
    }

    void resize(double xmin, double ymin, double xmax, double ymax){
        Point<Integer> imin = world2map(xmin, ymin);
        Point<Integer> imax=world2map(xmax, ymax);
        int pxmin, pymin, pxmax, pymax;
        pxmin=(int)Math.floor((float) imin.x / (1 << m_storage.getPatchMagnitude()));
        pxmax=(int)Math.ceil((float) imax.x / (1 << m_storage.getPatchMagnitude()));
        pymin=(int)Math.floor((float) imin.y / (1 << m_storage.getPatchMagnitude()));
        pymax=(int)Math.ceil((float) imax.y / (1 << m_storage.getPatchMagnitude()));
        m_storage.resize(pxmin, pymin, pxmax, pymax);
        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
        m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
    }

    void grow(double xmin, double ymin, double xmax, double ymax){
        Point<Integer> imin=world2map(xmin, ymin);
        Point<Integer> imax=world2map(xmax, ymax);
        if (isInside(imin) && isInside(imax))
            return;
        imin=Math.min(imin, new Point<Integer>(0,0));
        imax=Math.max(imax, Point<Integer>(m_mapSizeX-1,m_mapSizeY-1));
        int pxmin, pymin, pxmax, pymax;
        pxmin=(int)Math.floor((float) imin.x / (1 << m_storage.getPatchMagnitude()));
        pxmax=(int)Math.ceil((float) imax.x / (1 << m_storage.getPatchMagnitude()));
        pymin=(int)Math.floor((float) imin.y / (1 << m_storage.getPatchMagnitude()));
        pymax=(int)Math.ceil((float) imax.y / (1 << m_storage.getPatchMagnitude()));
        m_storage.resize(pxmin, pymin, pxmax, pymax);
        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
        m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
    }


    Point<Integer> world2map(Point<Double> p) {
        return new Point<Integer>( (int)Math.round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)Math.round((p.y-m_center.y)/m_delta)+m_sizeY2);
    }

    Point<Double> map2world(Point<Integer> p) {
        return new Point<Double>( (p.x-m_sizeX2)*m_delta + m_center.x,
                (p.y-m_sizeY2)*m_delta + m_center.y);
    }


    Cell cell(Point<Integer> p) {
        AccessibilityState s=m_storage.cellState(p);
        if (! s&Inside)
            assert(0);
        //if (s&Allocated) return m_storage.cell(p); assert(0);

        // this will never happend. Just to satify the compiler..
        return m_storage.cell(p);

    }


    Cell cell(Point p) {
        Point<Integer> ip=world2map(p);
        AccessibilityState s=m_storage.cellState(ip);
        if (! s&Inside)
            assert(0);
        //if (s&Allocated) return m_storage.cell(ip); assert(0);

        // this will never happend. Just to satify the compiler..
        return m_storage.cell(ip);
    }


    Cell cell(Point<Integer> p) const {
        AccessibilityState s=m_storage.cellState(p);
        //if (! s&Inside) assert(0);
        if (s&Allocated)
            return m_storage.cell(p);
        return m_unknown;
    }


    Cell cell(Point p) {
        Point<Integer> ip=world2map(p);
        AccessibilityState s=m_storage.cellState(ip);
        //if (! s&Inside) assert(0);
        if (s & Allocated)
            return m_storage.cell(ip);
        return  m_unknown;
    }


    //FIXME check why the last line of the map is corrupted.
    Array2D toDoubleArray() {
        Array2D darr= new Array2D(getMapSizeX()-1, getMapSizeY()-1);
        for(int x=0; x<getMapSizeX()-1; x++) {
            for (int y = 0; y < getMapSizeY() - 1; y++) {
                Point<Integer> p = new Point<Integer>(x, y);
                darr.cell(p) = cell(p);
            }
        }
        return darr;
    }



    GMap toDoubleMap() {
        //FIXME size the map so that m_center will be setted accordingly
        Point<Double> pmin = map2world(new Point<Integer>(0,0));
        Point<Double> pmax= map2world(new Point<Integer>(getMapSizeX()-1,getMapSizeY()-1));
        Point<Double> center = new Point<Double>((pmax.x + pmin.x) * 0.5, (pmax.y + pmin.y) * 0.5);
        GMap  plainMap=new GMap(center, (pmax.x-pmin.x), (pmax.x-pmin.y), getDelta());
        for(int x=0; x<getMapSizeX()-1; x++)
            for(int y=0; y<getMapSizeY()-1; y++){
                Point<Integer> p = new Point<Integer>(x,y);
                plainMap.cell(p) = cell(p);
            }
        return plainMap;
    }
}
