package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class TNode {
    /**The pose of the robot*/
    public OrientedPoint<Double> pose;

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

    public TNode(OrientedPoint<Double> p, double w, TNode n, int c) {
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
}
