package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;

public class ParticleMaps {
    private GMap map;

    private TNode node;

    private int index;

    private int task;

    public ParticleMaps() {
    }

    public ParticleMaps(GMap map, TNode node, int index, int task) {
        this.map = map;
        this.node = node;
        this.index = index;
        this.task = task;
    }

    public GMap getMap() {
        return map;
    }

    public TNode getNode() {
        return node;
    }

    public void setMap(GMap map) {
        this.map = map;
    }

    public void setNode(TNode node) {
        this.node = node;
    }
}
