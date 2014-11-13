package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.utils.OrientedPoint;

public class Particle {
    /** The map */
    public GMap map;
    /** The pose of the robot */
    public OrientedPoint<Double> pose;

    /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
    public OrientedPoint previousPose;

    /** The weight of the particle */
    public double weight;

    /** The cumulative weight of the particle */
    public double weightSum;

    public double gweight;

    /** The index of the previous particle in the trajectory tree */
    public int previousIndex;

    /** Entry to the trajectory tree */
    public TNode node;

    public Particle(GMap map) {
        this.map = map;
        pose = new OrientedPoint<Double>(0.0, 0.0);
        weight = 0;
        weightSum = 0;
        gweight = 0;
        previousIndex = 0;
        node = null;
    }

    public void setWeight(double weight) {
        this.weight = weight;
    }
}
