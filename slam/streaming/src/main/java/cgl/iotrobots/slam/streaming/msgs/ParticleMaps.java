package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.gridfastsalm.TNode;

import java.util.List;

public class ParticleMaps {
//    private StaticMap map;

    private TransferMap map;

    private int index;

    private int task;

    private List<TNodeValue> nodes;

    public ParticleMaps() {
    }

    public ParticleMaps(TransferMap map, int index, int task, List<TNodeValue> nodes) {
        this.map = map;
        this.index = index;
        this.task = task;
        this.nodes = nodes;
    }

    public TransferMap getMap() {
        return map;
    }

    public void setMap(TransferMap map) {
        this.map = map;
    }

    public int getIndex() {
        return index;
    }

    public int getTask() {
        return task;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public void setTask(int task) {
        this.task = task;
    }

    public List<TNodeValue> getNodes() {
        return nodes;
    }

    public void setNodes(List<TNodeValue> nodes) {
        this.nodes = nodes;
    }
}
