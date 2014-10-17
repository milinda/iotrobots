package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.sensor.RangeReading;
import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class TNode {
    /**The pose of the robot*/
    OrientedPoint pose;

    /**The weight of the particle*/
    double weight;

    /**The sum of all the particle weights in the previous part of the trajectory*/
    double accWeight;

    double gweight;

    /**The parent*/
    TNode parent;

    /**The range reading to which this node is associated*/
    RangeReading reading;

    /**The number of childs*/
    int childs;

    /**counter in visiting the node (internally used)*/
    int visitCounter;

    /**visit flag (internally used)*/
    boolean flag;
}
