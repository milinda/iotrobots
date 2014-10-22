package cgl.iotrobots.slam.core.grid;

import cgl.iotrobots.slam.core.utils.Point;

public class GMap <Cell, Storage> {
    Point<Double> m_center;
    double m_worldSizeX, m_worldSizeY, m_delta;
    Storage m_storage;
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
        m_center=center;
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_delta=delta;
        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_sizeX2=(int)round((m_center.x-xmin)/m_delta);
        m_sizeY2=(int)round((m_center.y-ymin)/m_delta);
    }

    public GMap(Point center, double xmin, double ymin, double xmax, double ymax, double delta) {
        m_center=center;
        m_worldSizeX=worldSizeX;
        m_worldSizeY=worldSizeY;
        m_delta=delta;
        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_sizeX2=m_mapSizeX>>1;
        m_sizeY2=m_mapSizeY>>1;
    }

    Map<Cell,Storage,isClass>::Map(int mapSizeX, int mapSizeY, double delta):
    m_storage(mapSizeX, mapSizeY){

    }

    template <class Cell, class Storage, const bool isClass>
    Map<Cell,Storage,isClass>::Map(const Point& center, double worldSizeX, double worldSizeY, double delta):
    m_storage((int)ceil(worldSizeX/delta), (int)ceil(worldSizeY/delta)){

    }

    template <class Cell, class Storage, const bool isClass>
    Map<Cell,Storage,isClass>::Map(const Point& center, double xmin, double ymin, double xmax, double ymax, double delta):
    m_storage((int)ceil((xmax-xmin)/delta), (int)ceil((ymax-ymin)/delta)){

    }

    void resize(double xmin, double ymin, double xmax, double ymax){
        Point<Integer> imin = world2map(xmin, ymin);
        Point<Integer> imax=world2map(xmax, ymax);
        int pxmin, pymin, pxmax, pymax;
        pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
        pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
        pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
        pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
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
        pxmin=(int)floor((float)imin.x/(1<<m_storage.getPatchMagnitude()));
        pxmax=(int)ceil((float)imax.x/(1<<m_storage.getPatchMagnitude()));
        pymin=(int)floor((float)imin.y/(1<<m_storage.getPatchMagnitude()));
        pymax=(int)ceil((float)imax.y/(1<<m_storage.getPatchMagnitude()));
        m_storage.resize(pxmin, pymin, pxmax, pymax);
        m_mapSizeX=m_storage.getXSize()<<m_storage.getPatchSize();
        m_mapSizeY=m_storage.getYSize()<<m_storage.getPatchSize();
        m_worldSizeX=xmax-xmin;
        m_worldSizeY=ymax-ymin;
        m_sizeX2-=pxmin*(1<<m_storage.getPatchMagnitude());
        m_sizeY2-=pymin*(1<<m_storage.getPatchMagnitude());
    }


    Point<Integer> world2map(Point& p) const{
        return new Point<Integer>( (int)round((p.x-m_center.x)/m_delta)+m_sizeX2, (int)round((p.y-m_center.y)/m_delta)+m_sizeY2);
    }

    Point map2world(Point<Integer>& p) const{
        return Point( (p.x-m_sizeX2)*m_delta,
                (p.y-m_sizeY2)*m_delta)+m_center;
    }


    template <class Cell, class Storage, const bool isClass>
    Cell& Map<Cell,Storage,isClass>::cell(const IntPoint& p) {
        AccessibilityState s=m_storage.cellState(p);
        if (! s&Inside)
            assert(0);
        //if (s&Allocated) return m_storage.cell(p); assert(0);

        // this will never happend. Just to satify the compiler..
        return m_storage.cell(p);

    }


    Cell& Map<Cell,Storage,isClass>::cell(const Point& p) {
        IntPoint ip=world2map(p);
        AccessibilityState s=m_storage.cellState(ip);
        if (! s&Inside)
            assert(0);
        //if (s&Allocated) return m_storage.cell(ip); assert(0);

        // this will never happend. Just to satify the compiler..
        return m_storage.cell(ip);
    }


            const Cell& Map<Cell,Storage,isClass>::cell(const IntPoint& p) const {
        AccessibilityState s=m_storage.cellState(p);
        //if (! s&Inside) assert(0);
        if (s&Allocated)
            return m_storage.cell(p);
        return m_unknown;
    }


            const  Cell& Map<Cell,Storage,isClass>::cell(const Point& p) const {
        IntPoint ip=world2map(p);
        AccessibilityState s=m_storage.cellState(ip);
        //if (! s&Inside) assert(0);
        if (s&Allocated)
            return m_storage.cell(ip);
        return  m_unknown;
    }


//FIXME check why the last line of the map is corrupted.

    DoubleArray2D* toDoubleArray() {
        DoubleArray2D* darr=new DoubleArray2D(getMapSizeX()-1, getMapSizeY()-1);
        for(int x=0; x<getMapSizeX()-1; x++)
            for(int y=0; y<getMapSizeY()-1; y++){
                IntPoint p(x,y);
                darr->cell(p)=cell(p);
            }
        return darr;
    }



    Map<double, DoubleArray2D, false>* toDoubleMap() const{
//FIXME size the map so that m_center will be setted accordingly
        Point pmin=map2world(IntPoint(0,0));
        Point pmax=map2world(getMapSizeX()-1,getMapSizeY()-1);
        Point center=(pmax+pmin)*0.5;
        Map<double, DoubleArray2D, false>*  plainMap=new Map<double, DoubleArray2D, false>(center, (pmax-pmin).x, (pmax-pmin).y, getDelta());
        for(int x=0; x<getMapSizeX()-1; x++)
            for(int y=0; y<getMapSizeY()-1; y++){
                IntPoint p(x,y);
                plainMap->cell(p)=cell(p);
            }
        return plainMap;
    }
}
