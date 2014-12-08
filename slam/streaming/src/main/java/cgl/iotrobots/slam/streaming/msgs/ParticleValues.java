package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.gridfastsalm.TNode;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;

/**
 * Particle without the map. This will be emited by the ScanMatchBolt,
 * The resampling bolt will use these values to choose the best particles.
 */
public class ParticleValues {
    /** the task that sent the particle */
    public int taskId;
    /** The index of the particle */
    public int index;
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

    private int totalTasks;

    public ParticleValues() {
    }

    public ParticleValues(int taskId, int index, int totalTasks, DoubleOrientedPoint pose,
                          DoubleOrientedPoint previousPose, double weight,
                          double weightSum, double gweight, int previousIndex,
                          TNode node) {
        this.taskId = taskId;
        this.index = index;
        this.pose = pose;
        this.previousPose = previousPose;
        this.weight = weight;
        this.weightSum = weightSum;
        this.gweight = gweight;
        this.previousIndex = previousIndex;
        this.node = node;
        this.totalTasks = totalTasks;
    }

    public int getTotalTasks() {
        return totalTasks;
    }

    public void setTaskId(int taskId) {
        this.taskId = taskId;
    }

    public int getTaskId() {
        return taskId;
    }

    public void setTotalTasks(int totalTasks) {
        this.totalTasks = totalTasks;
    }

    public int getIndex() {
        return index;
    }

    public DoubleOrientedPoint getPose() {
        return pose;
    }

    public DoubleOrientedPoint getPreviousPose() {
        return previousPose;
    }

    public double getWeight() {
        return weight;
    }

    public double getWeightSum() {
        return weightSum;
    }

    public double getGweight() {
        return gweight;
    }

    public int getPreviousIndex() {
        return previousIndex;
    }

    public TNode getNode() {
        return node;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public void setPose(DoubleOrientedPoint pose) {
        this.pose = pose;
    }

    public void setPreviousPose(DoubleOrientedPoint previousPose) {
        this.previousPose = previousPose;
    }

    public void setWeight(double weight) {
        this.weight = weight;
    }

    public void setWeightSum(double weightSum) {
        this.weightSum = weightSum;
    }

    public void setGweight(double gweight) {
        this.gweight = gweight;
    }

    public void setPreviousIndex(int previousIndex) {
        this.previousIndex = previousIndex;
    }

    public void setNode(TNode node) {
        this.node = node;
    }
}
