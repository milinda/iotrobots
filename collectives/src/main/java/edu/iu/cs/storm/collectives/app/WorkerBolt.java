package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class WorkerBolt extends BaseRichBolt {
    private TopologyContext context;
    private OutputCollector collector;
    private int taskId = 0;

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.context = context;
        this.taskId = context.getThisTaskId();
    }

    @Override
    public void execute(Tuple tuple) {
        long receiveTime = System.currentTimeMillis();
        Object body = tuple.getValueByField(Constants.Fields.BODY);

        Trace trace = (Trace) tuple.getValueByField(Constants.Fields.TRACE_FIELD);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        trace.setTaskId(taskId);
        trace.setBcastReceiveTime(receiveTime);

        List<Object> list = new ArrayList<Object>();
        list.add(body);
        list.add(trace);
        collector.emit(Constants.Fields.DATA_STREAM, list);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.DATA_STREAM, new Fields(Constants.Fields.DATA_FIELD,
                Constants.Fields.TRACE_FIELD));
    }
}
