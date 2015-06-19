package cgl.iotrobots.collavoid.commons.iotcloud;


import backtype.storm.generated.GlobalStreamId;
import backtype.storm.grouping.CustomStreamGrouping;
import backtype.storm.task.WorkerTopologyContext;

import java.util.Arrays;
import java.util.List;

public class ModStreamGrouping implements CustomStreamGrouping {

    private List<Integer> _targetTasks;

    public ModStreamGrouping() {
    }

    @Override
    public void prepare(WorkerTopologyContext workerTopologyContext, GlobalStreamId globalStreamId, List<Integer> list) {
        _targetTasks = list;
    }

    @Override
    public List<Integer> chooseTasks(int i, List<Object> list) {
        Long groupingKey = Long.valueOf(list.get(list.size() - 1).toString());
        int index = (int) (groupingKey % (_targetTasks.size()));
        return Arrays.asList(_targetTasks.get(index));
    }
}
