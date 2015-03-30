package cgl.iotrobots.slam.streaming.msgs;

public class Ready {
    private int taskId;

    public Ready() {
    }

    public Ready(int taskId) {
        this.taskId = taskId;
    }

    public int getTaskId() {
        return taskId;
    }

    public void setTaskId(int taskId) {
        this.taskId = taskId;
    }
}
