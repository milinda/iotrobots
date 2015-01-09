package cgl.iotrobots.collavoid.commons.planners;


import java.io.Serializable;

public class LinePair implements Serializable {
    private Vector2 first;
    private Vector2 second;

    public LinePair(Vector2 first,Vector2 second){
        setFirst(first);
        setSecond(second);
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


    public static Vector2 Intersection(LinePair l1,LinePair l2){
        double r, s, d;
        double x1=l1.getFirst().getX(),x2=l1.getSecond().getX(),x3=l2.getFirst().getX(),x4=l2.getSecond().getX();
        double y1=l1.getFirst().getY(),y2=l1.getSecond().getY(),y3=l2.getFirst().getY(),y4=l2.getSecond().getY();
        Vector2 res=null;
        //Make sure the lines aren't parallel
        if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3)){
            d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
            if (d != 0){
                r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
                s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
                if (r >= 0 && r <= 1){
                    if (s >= 0 && s <= 1){
                        return new Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
                    }
                }
            }
        }
        return res;
    }

}
