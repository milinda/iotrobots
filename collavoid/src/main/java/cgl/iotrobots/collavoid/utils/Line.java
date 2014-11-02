package cgl.iotrobots.collavoid.utils;


/**
 * Created by hjh on 10/24/14.
 */
public class Line {
    private Vector2 point;
    private Vector2 dir;

    public Line(Vector2 point, Vector2 dir) {
        this.point = new Vector2(point);
        this.dir = new Vector2(dir);
    }

    public Line(){
        this.point=new Vector2(0,0);
        this.dir=new Vector2(0,0);
    }

    public void setPoint(Vector2 p){
        this.point=new Vector2(p);
    }

    public void setDir(Vector2 d){
        this.dir=new Vector2(d);
    }

}

