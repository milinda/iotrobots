package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import com.esotericsoftware.kryo.Kryo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class BroadCastBolt extends BaseRichBolt {
    private TopologyContext context;
    private OutputCollector collector;
    private Kryo kryo;
    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.context = context;
        this.collector = collector;
        this.kryo = new Kryo();
        Utils.registerClasses(kryo);
    }

    @Override
    public void execute(Tuple tuple) {
        String stream = tuple.getSourceStreamId();
        if (stream.equals(Constants.Fields.CONTROL_STREAM)) {
            return;
        }

        Object body = tuple.getValueByField(Constants.Fields.BODY);
        Object time = tuple.getValueByField(Constants.Fields.TIME_FIELD);
        Trace trace = new Trace();
        trace.setTime(Long.parseLong(time.toString()));

        byte []b = Utils.serialize(kryo, trace);
        List<Object> list = new ArrayList<Object>();
        list.add(body);
        list.add(b);
        collector.emit(Constants.Fields.BROADCAST_STREAM, list);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declareStream(Constants.Fields.BROADCAST_STREAM, new Fields(
                Constants.Fields.BODY,
                Constants.Fields.TRACE_FIELD));
    }
}
