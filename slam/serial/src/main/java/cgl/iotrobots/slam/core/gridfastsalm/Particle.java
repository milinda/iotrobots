package cgl.iotrobots.slam.core.gridfastsalm;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.grid.HierarchicalArray2D;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

public class Particle {
    /** The map */
    public GMap map;
    /** The pose of the robot */
    public DoubleOrientedPoint pose;

    /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
    public DoubleOrientedPoint previousPose;

    /** The weight of the particle */
    public double weight;

    /** The cumulative weight of the particle */
    public double weightSum;

    public double gweight;

    /** The index of the previous particle in the trajectory tree */
    public int previousIndex;

    /** Entry to the trajectory tree */
    public TNode node;

    public Particle(Particle p) {
        map = p.map;
        map.m_storage = new HierarchicalArray2D(p.map.m_storage);
        pose = new DoubleOrientedPoint(p.pose);
        previousPose = new DoubleOrientedPoint(p.previousPose);
        weight = p.weight;
        weightSum = p.weightSum;
        gweight = p.gweight;
        previousIndex = p.previousIndex;
        node = p.node;
    }

    public Particle(GMap map) {
        this.map = map;
        pose = new DoubleOrientedPoint(0.0, 0.0);
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
