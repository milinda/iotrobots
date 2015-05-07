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

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.context = context;
    }

    @Override
    public void execute(Tuple tuple) {
        Object body = tuple.getValueByField(Constants.Fields.BODY);

        Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        Object sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        List<Object> list = new ArrayList<Object>();
        list.add(body);
        list.add(sensorId);
        list.add(time);
        list.add(new Trace());
        collector.emit(Constants.Fields.DATA_STREAM, list);
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
