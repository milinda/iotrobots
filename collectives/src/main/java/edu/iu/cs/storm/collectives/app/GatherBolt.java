package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import com.esotericsoftware.kryo.Kryo;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class GatherBolt extends BaseRichBolt {
    private int workers = 0;
    private TopologyContext topologyContext;
    private OutputCollector collector;

    private Kryo kryo;
    private Trace currentTrace;
    private List<Tuple> inputs = new ArrayList<Tuple>();

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.topologyContext = context;
        this.workers = context.getComponentTasks(Constants.Topology.WORKER_BOLT).size();
        this.kryo = new Kryo();
        Utils.registerClasses(this.kryo);
        this.currentTrace = new Trace();
    }


    @Override
    public void execute(Tuple tuple) {
        String stream = tuple.getSourceStreamId();
        if (stream.equals(Constants.Fields.CONTROL_STREAM)) {
            return;
        }

        // wait until we get all the inputs
        processInput(tuple);
        inputs.add(tuple);

        if (inputs.size() >= workers) {
            // emit the current trace
            byte []b = Utils.serialize(kryo, currentTrace);
            List<Object> list = new ArrayList<Object>();
            list.add(b);
            list.add("test");
            list.add(currentTrace.getTime());

            collector.emit(Constants.Fields.SEND_STREAM, list);

            this.inputs.clear();
            this.currentTrace = new Trace();
        }
    }

    private void processInput(Tuple tuple) {
        Object body = tuple.getValueByField(Constants.Fields.DATA_FIELD);
        byte []traceBytes = (byte[]) tuple.getValueByField(Constants.Fields.TRACE_FIELD);
        Trace trace = (Trace) Utils.deSerialize(kryo, traceBytes, Trace.class);
        processTrace(trace);
    }

    private void processTrace(Trace trace) {
        System.out.println("Got trace: " + trace.getTaskId());
        currentTrace.setTime(trace.getTime());
        currentTrace.getBcastReceiveTimes().put(trace.getTaskId(), trace.getBcastReceiveTime());
        currentTrace.getGatherReceiveTimes().put(trace.getTaskId(), System.currentTimeMillis());
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.SEND_STREAM,
                new Fields("body", Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
    }
}
