package cgl.iotrobots.collavoid.utils;

/**
 * Created by hjh on 10/25/14.
 */
public class LinePair {
    private Vector2 first;
    private Vector2 second;

    public LinePair(Vector2 first,Vector2 second){
        this.first=new Vector2(first);
        this.second=new Vector2(second);
    }

//    public LinePair(LinePair lp){
//        this.first=new Vector2(lp.getFirst());
//        this.second=new Vector2(lp.getSecond());
//    }

    public LinePair(){
        this.first=new Vector2(0,0);
        this.second=new Vector2(0,0);
    }

    public void setFirst(Vector2 fst) {
        first = new Vector2(fst);
    }

    public void setSecond(Vector2 sec) {
        second = new Vector2(sec);
    }

    public void setPair(Vector2 fst, Vector2 sec) {
        first = new Vector2(fst);
        second = new Vector2(sec);
    }

    public Vector2 getFirst(){
        return new Vector2(this.first);
    }

    public Vector2 getSecond(){
        return new Vector2(this.second);
    }
}
