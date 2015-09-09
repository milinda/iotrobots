package edu.iu.cs.storm.collectives.app;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GatherBolt extends BaseRichBolt {
    private Logger LOG = LoggerFactory.getLogger(GatherBolt.class);
    private int workers = 0;
    private TopologyContext topologyContext;
    private OutputCollector collector;

    private Kryo kryo;
    private BTrace currentTrace;
    private Map<String, List<Tuple>> inputs = new HashMap<String, List<Tuple>>();

    @Override
    public void prepare(Map stormConf, TopologyContext context, OutputCollector collector) {
        this.collector = collector;
        this.topologyContext = context;
        this.workers = context.getComponentTasks(Constants.Topology.WORKER_BOLT).size();
        this.kryo = new Kryo();
        Utils.registerClasses(this.kryo);
        this.currentTrace = new BTrace();
    }


    @Override
    public void execute(Tuple tuple) {
        String stream = tuple.getSourceStreamId();
        if (stream.equals(Constants.Fields.CONTROL_STREAM)) {
            return;
        }

        // wait until we get all the inputs
        processInput(tuple);

        String messageId = tuple.getStringByField(Constants.Fields.MESSAGE_ID_FIELD);

        if(!inputs.containsKey(messageId)) {
            ArrayList<Tuple> tuples = new ArrayList<Tuple>();
            inputs.put(messageId, tuples);
        }

        inputs.get(messageId).add(tuple);

        if (inputs.get(messageId).size() >= workers) {
            inputs.remove(messageId);
            LOG.info("Gather done for message with ID: " + messageId);
            // emit the current trace
            byte []b = Utils.serialize(kryo, currentTrace);
            List<Object> list = new ArrayList<Object>();
            list.add(b);
            list.add("test");
            list.add(currentTrace.getTime());

            collector.emit(Constants.Fields.SEND_STREAM, list);

            this.inputs.clear();
            this.currentTrace = new BTrace();

            list = new ArrayList<Object>();
            list.add("ready");
            collector.emit(Constants.Fields.READY_STREAM, list);
        }
    }

    private void processInput(Tuple tuple) {
        Object body = tuple.getValueByField(Constants.Fields.DATA_FIELD);
        byte []traceBytes = (byte[]) tuple.getValueByField(Constants.Fields.TRACE_FIELD);
        BTrace trace = (BTrace) Utils.deSerialize(kryo, traceBytes, BTrace.class);
        processTrace(trace);
    }

    private void processTrace(BTrace trace) {
        LOG.info("Got trace: " + trace.getTaskId());
        currentTrace.setTime(trace.getTime());
        currentTrace.getBcastReceiveTimes().put(trace.getTaskId(), trace.getBcastReceiveTime());
        currentTrace.getGatherReceiveTimes().put(trace.getTaskId(), System.currentTimeMillis());
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constants.Fields.SEND_STREAM,
                new Fields("body", Constants.Fields.SENSOR_ID_FIELD, Constants.Fields.TIME_FIELD));
        outputFieldsDeclarer.declareStream(Constants.Fields.READY_STREAM,
                new Fields(Constants.Fields.DATA_FIELD));
    }
}
