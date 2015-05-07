package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;

import java.util.Map;

public class GatherBolt extends BaseRichBolt {
    private int workers = 0;
    private TopologyContext topologyContext;
    private OutputCollector collector;

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.topologyContext = context;
        this.workers = context.getComponentTasks(Constants.Topology.WORKER_BOLT).size();
    }

    @Override
    public void execute(Tuple input) {
        // wait until we gel all

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.SEND_STREAM,
                new Fields("body", Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
    }
}
