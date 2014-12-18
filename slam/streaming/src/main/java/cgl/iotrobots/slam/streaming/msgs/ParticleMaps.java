package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.grid.GMap;
import cgl.iotrobots.slam.core.gridfastsalm.TNode;

public class ParticleMaps {
    private TransferMap map;

    private int index;

    private int task;

    public ParticleMaps() {
    }

    public ParticleMaps(TransferMap map, int index, int task) {
        this.map = map;
        this.index = index;
        this.task = task;
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
}
