package edu.iu.cs.storm.collectives.app;

import java.util.HashMap;
import java.util.Map;

public class Trace {
    private Map<Integer, Long> gatherReceiveTimes = new HashMap<Integer, Long>();

    private Map<Integer, Long> bcastReceiveTimes = new HashMap<Integer, Long>();

    private int taskId;

    private long time;

    private long bcastReceiveTime = 0;

    public long getBcastReceiveTime() {
        return bcastReceiveTime;
    }

    public void setBcastReceiveTime(long bcastReceiveTime) {
        this.bcastReceiveTime = bcastReceiveTime;
    }

    public long getTime() {
        return time;
    }

    public void setTime(long time) {
        this.time = time;
    }

    public Map<Integer, Long> getGatherReceiveTimes() {
        return gatherReceiveTimes;
    }

    public Map<Integer, Long> getBcastReceiveTimes() {
        return bcastReceiveTimes;
    }

    public int getTaskId() {
        return taskId;
    }

    public void setGatherReceiveTimes(Map<Integer, Long> gatherReceiveTimes) {
        this.gatherReceiveTimes = gatherReceiveTimes;
    }

    public void setBcastReceiveTimes(Map<Integer, Long> bcastReceiveTimes) {
        this.bcastReceiveTimes = bcastReceiveTimes;
    }

    public void setTaskId(int taskId) {
        this.taskId = taskId;
    }

    public String serialize() {
        String s = "'";
        String g = "'";
        int size = gatherReceiveTimes.size();
        for (int i = 0; i < size; i++) {
            s += gatherReceiveTimes.get(i) + " ";
            g += bcastReceiveTimes.get(i) + " ";
        }
        s += "'";
        g += "'";
        return time + ", " + s + " ," + g;
    }
}
