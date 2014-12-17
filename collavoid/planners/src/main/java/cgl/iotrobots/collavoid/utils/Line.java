package cgl.iotrobots.collavoid.utils;



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

    public Vector2 getPoint(){
        return new Vector2(this.point);
    }

    public Vector2 getDir(){
        return new Vector2(this.dir);
    }

    public void setPoint(Vector2 p){
        this.point=new Vector2(p);
    }

    public void setDir(Vector2 d){
        this.dir=new Vector2(d);
    }


}

