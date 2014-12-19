package cgl.iotrobots.collavoid.commons.planners;

import geometry_msgs.Point32;


public class Vector2 {
    private double x;
    private double y;

    public Vector2() {
        this.x = 0.0;
        this.y = 0.0;
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 p) {
        this.x = p.getX();
        this.y = p.getY();
    }


    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setVector2(Vector2 p) {
        this.x = p.getX();
        this.y = p.getY();
    }

    public double getLength(){
        return Vector2.abs(this);
    }


    public static Vector2 negative(Vector2 v) {
        return new Vector2(-v.x, -v.y);
    }

    public static Vector2 minus(Vector2 p1, Vector2 p2) {
        double x = p1.getX() - p2.getX();
        double y = p1.getY() - p2.getY();
        return new Vector2(x, y);
    }

    public void minus(Vector2 p1) {
        this.x -= p1.getX();
        this.y -= p1.getX();
    }

    public static Vector2 plus(Vector2 p1, Vector2 p2) {
        double x = p1.getX() + p2.getX();
        double y = p1.getY() + p2.getY();
        return new Vector2(x, y);
    }

    public void plus(Vector2 p1) {
        this.x += p1.getX();
        this.y += p1.getY();
    }

    public void mul(Vector2 p1) {
        this.x *= p1.getX();
        this.y *= p1.getY();
    }

    public static Vector2 mul(Vector2 v1, double s) {
        return new Vector2(v1.x * s, v1.y * s);
    }

    public void scale(double s) {
        this.x *= s;
        this.y *= s;
    }


    public static double dotProduct(Vector2 p1, Vector2 p2) {
        double x = p1.getX() * p2.getX();
        double y = p1.getY() * p2.getY();
        return x + y;
    }

    public static double det(Vector2 v1, Vector2 v2) {
        return det(v1.getX(),v1.getY(),v2.getX(),v2.getY());
    }

    public static double det(double x1,double y1,double x2,double y2){
        return x1*y2-y1*x2;
    }

    public static double absSqr(double x, double y) {
        double x1 = x * x;
        double y1 = y * y;
        return x1 + y1;
    }

    public static double absSqr(Vector2 p1) {
        double x = p1.getX() * p1.getX();
        double y = p1.getY() * p1.getY();
        return x + y;
    }

    public static double abs(double x, double y) {
        return Math.sqrt(absSqr(x, y));
    }

    public static double abs(Vector2 p1) {
        return Math.sqrt(Vector2.absSqr(p1));
    }

    @Override
    public boolean equals(Object obj) {
        if(!(obj instanceof Vector2))
            return false;
        Vector2 v2=(Vector2)obj;
        if (this.x == v2.getX() && this.y == v2.getY()) {
            return true;
        } else {
            return false;
        }
    }



    public static Vector2 normalize(Vector2 p) {
        double x = p.getX() / abs(p);
        double y = p.getY() / abs(p);
        return new Vector2(x, y);
    }

    //calculate normal vector of p, clockwise
    public static Vector2 normal(Vector2 p){
        return normalize(new Vector2(p.getY(),-p.getX()));
    }

    //convert pointcloud point32 datatype to Point2
    public static Vector2 Point32ToVector2(Point32 p) {
        double x = p.getX();
        double y = p.getY();
        return new Vector2(x, y);
    }

    public static Vector2 rotateVectorByAngle(Vector2 p, double ang) {
        double cos_a, sin_a, x, y;
        cos_a = Math.cos(ang);
        sin_a = Math.sin(ang);
        x = cos_a * p.getX() - sin_a * p.getY();
        y = cos_a * p.getY() + sin_a * p.getX();
        return new Vector2(x, y);
    }

    public static double angleBetween(final Vector2 one, final Vector2 two) {
        double dot_prod = Vector2.dotProduct(one,two);
        double len1 = abs(one);
        double len2 = abs(two);
        return Math.acos(dot_prod / (len1*len2));
    }

    public static double atan(Vector2 v){
        return Math.atan2(v.getY(),v.getX());
    }
}
