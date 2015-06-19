package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.planners.AgentCtlPubState;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class DispatcherBolt extends BaseRichBolt {
    Logger logger = LoggerFactory.getLogger(DispatcherBolt.class);
    OutputCollector collector;
    Map<String, AgentCtlPubState> timeMap = new ConcurrentHashMap<String, AgentCtlPubState>();
    Map<String, Integer> cntMap = new ConcurrentHashMap<String, Integer>();
    Map<String, String> idxMap = new ConcurrentHashMap<String, String>();

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        if (tuple.getSourceComponent().equals(Constant_storm.Components.TIMER_SPOUT_COMPONENT)) {
            checkNewEmit();
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.GLOBAL_PLANNER_COMPONENT)) {
            cacheTimes(tuple);
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM)) {
            switchCtlState(tuple);
        } else if (tuple.getSourceStreamId().equals(Constant_storm.Streams.PUBLISHME_STREAM)) {
            switchPubState(tuple);
        }
        collector.ack(tuple);
    }

    private void switchPubState(Tuple tuple) {
        String id = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        if (timeMap.containsKey(id) && timeMap.get(id).publishing) {
            timeMap.get(id).publishing = false;
        }
    }

    private void switchCtlState(Tuple tuple) {
        String id = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        if ((Boolean) tuple.getValueByField(Constant_storm.FIELDS.AGENT_STATE_FIELD)) {
            timeMap.remove(id);
            idxMap.remove(id);
        }
        if (timeMap.containsKey(id) && timeMap.get(id).controlling) {
            timeMap.get(tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD)).controlling = false;
        }
    }

    private void cacheTimes(Tuple tuple) {
        String id = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        // only used for the first time
        if (!cntMap.containsKey(id))
            cntMap.put(id, new Integer(0));
        AgentCtlPubState state = (AgentCtlPubState) tuple.getValueByField(Constant_storm.FIELDS.CTL_PUB_TIME_FIELD);
        timeMap.put(id, state);
        // for grouping
        idxMap.put(id, tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD).toString());
        collector.emit(Constant_storm.Streams.ACK_STREAM, new Values(
                id,
                state.seq,
                tuple.getValueByField(Constant_storm.FIELDS.AGENT_IDX_FIELD)));
    }

    private void checkNewEmit() {
        long now;

        for (Map.Entry<String, AgentCtlPubState> e : timeMap.entrySet()) {
            AgentCtlPubState state = e.getValue();
            now = System.currentTimeMillis();
            String idx = idxMap.get(e.getKey());
            if (!state.publishing && (now - state.lastTimeMePublished > state.publishMePeriod * 1000 - 5)) {
                state.lastTimeMePublished = now;
                state.publishing = true;
//              record delay, not used
                String emitTime = "sl," + System.currentTimeMillis() + ',';
                collector.emit(Constant_storm.Streams.PUBLISHME_STREAM, new Values(
                        now,
                        e.getKey(),
                        emitTime,
                        idx
                ));
            }

            if (cntMap.get(e.getKey()) < 50) {
                // establish stable state
                cntMap.put(e.getKey(), new Integer(cntMap.get(e.getKey()) + 1));
            } else {
                if (!state.controlling && (now - state.lastTimeControlled > state.controlPeriod * 1000 - 5)) {
                    state.lastTimeControlled = now;
                    state.controlling = true;
                    String emitTime = "cl," + System.currentTimeMillis() + ',';
                    collector.emit(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Values(
                            now,
                            e.getKey(),
                            emitTime,
                            idxMap.get(e.getKey())
                    ));
                }
            }
        }

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CALCULATE_VELOCITY_CMD_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.EMIT_TIME_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISHME_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.EMIT_TIME_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.ACK_STREAM, new Fields(
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.SEQUENCE_FIELD,
                Constant_storm.FIELDS.AGENT_IDX_FIELD
        ));
    }


}
