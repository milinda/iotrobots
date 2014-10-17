package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.scanmatcher.ScanMatcherMap;
import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class Particle {
    /** The map */
    ScanMatcherMap map;
    /** The pose of the robot */
    OrientedPoint pose;

    /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
    OrientedPoint previousPose;

    /** The weight of the particle */
    double weight;

    /** The cumulative weight of the particle */
    double weightSum;

    double gweight;

    /** The index of the previous particle in the trajectory tree */
    int previousIndex;

    /** Entry to the trajectory tree */
    TNode node;
}
