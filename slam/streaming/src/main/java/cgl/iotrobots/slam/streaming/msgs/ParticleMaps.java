package cgl.iotrobots.slam.streaming.msgs;

import cgl.iotrobots.slam.core.grid.StaticMap;

public class ParticleMaps {
    private StaticMap map;

    private int index;

    private int task;

    public ParticleMaps() {
    }

    public ParticleMaps(StaticMap map, int index, int task) {
        this.map = map;
        this.index = index;
        this.task = task;
    }

    public StaticMap getMap() {
        return map;
    }

    public void setMap(StaticMap map) {
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
