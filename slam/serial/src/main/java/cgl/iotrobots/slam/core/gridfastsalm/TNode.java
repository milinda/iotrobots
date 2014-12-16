package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

public class TNode {
    /**The pose of the robot*/
    public DoubleOrientedPoint pose;

    /**The weight of the particle*/
    public double weight;

    /**The sum of all the particle weights in the previous part of the trajectory*/
    public double accWeight;

    public double gweight;

    /**The parent*/
    public TNode parent;

    /**The range reading to which this node is associated*/
    public RangeReading reading;

    /**The number of childs*/
    public int childs;

    /**counter in visiting the node (internally used)*/
    public int visitCounter;

    /**visit flag (internally used)*/
    public boolean flag;

    public TNode(DoubleOrientedPoint p, double w, TNode n, int c) {
        pose = p;
        weight = w;
        childs = c;
        parent = n;
        reading = null;
        gweight = 0;
        if (n != null) {
            n.childs++;
        }
        flag = false;
        accWeight = 0;
    }

    public TNode(TNode t) {

    }

    public TNode() {
    }

    public DoubleOrientedPoint getPose() {
        return pose;
    }

    public double getWeight() {
        return weight;
    }

    public double getAccWeight() {
        return accWeight;
    }

    public double getGweight() {
        return gweight;
    }

    public TNode getParent() {
        return parent;
    }

    public RangeReading getReading() {
        return reading;
    }

    public int getChilds() {
        return childs;
    }

    public int getVisitCounter() {
        return visitCounter;
    }

    public boolean isFlag() {
        return flag;
    }

    public void setPose(DoubleOrientedPoint pose) {
        this.pose = pose;
    }

    public void setWeight(double weight) {
        this.weight = weight;
    }

    public void setAccWeight(double accWeight) {
        this.accWeight = accWeight;
    }

    public void setGweight(double gweight) {
        this.gweight = gweight;
    }

    public void setParent(TNode parent) {
        this.parent = parent;
    }

    public void setReading(RangeReading reading) {
        this.reading = reading;
    }

    public void setChilds(int childs) {
        this.childs = childs;
    }

    public void setVisitCounter(int visitCounter) {
        this.visitCounter = visitCounter;
    }

    public void setFlag(boolean flag) {
        this.flag = flag;
    }
}
