package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;

import java.util.Map;

public class WorkerBolt extends BaseRichBolt {
    private TopologyContext context;
    private OutputCollector collector;

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.context = context;
    }

    @Override
    public void execute(Tuple input) {

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.GATHER_STREAM, new Fields(Constants.Fields.DATA_FIELD,
                Constants.Fields.DATA_FIELD,
                Constants.Fields.SENSOR_ID_FIELD,
                Constants.Fields.TIME_FIELD,
                Constants.Fields.TRACE_FIELD));
    }
}
