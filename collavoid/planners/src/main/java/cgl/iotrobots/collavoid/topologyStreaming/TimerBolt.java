package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.BaseConfig_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.HashMap;
import java.util.Map;

public class TimerBolt extends BaseBasicBolt {
    private Map<String, TimerboltContext> contextMap = new HashMap<String, TimerboltContext>();
    private long now;

    private class TimerboltContext {
        long pubMePeriod;
        long controlPeriod;
        long lastMePub;
        long lastControlled;
    }

    @Override
    public void execute(Tuple tuple, BasicOutputCollector basicOutputCollector) {
        if (tuple.getSourceComponent().equals(Constant_storm.Components.TIMER_SPOUT_COMPONENT)) {
            now = System.currentTimeMillis();
            for (Map.Entry<String, TimerboltContext> e : contextMap.entrySet()) {
                if (now - e.getValue().lastMePub > e.getValue().pubMePeriod) {
                    e.getValue().lastMePub = now;
                    basicOutputCollector.emit(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM,
                            new Values(now, e.getKey()));
                }
                if (now - e.getValue().lastControlled > e.getValue().controlPeriod) {
                    e.getValue().lastControlled = now;
                    basicOutputCollector.emit(Constant_storm.Streams.CONTROLLER_TIMER_STREAM,
                            new Values(now, e.getKey()));
                }
            }
        } else if (tuple.getSourceComponent().equals(Constant_storm.Components.BASE_CONFIG_COMPONENT)) {
            String sensorID = tuple.getStringByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            BaseConfig_ conf = (BaseConfig_) tuple.getValueByField(Constant_storm.FIELDS.BASE_CONFIG_FIELD);
            TimerboltContext context = new TimerboltContext();
            context.pubMePeriod = (long) (1 / conf.getPublisMeFreq() * 1000);
            context.controlPeriod = (long) (1 / conf.getControlFreq() * 1000);
            contextMap.put(sensorID, context);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM,
                new Fields(Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CONTROLLER_TIMER_STREAM,
                new Fields(Constant_storm.FIELDS.TIME_FIELD,
                        Constant_storm.FIELDS.SENSOR_ID_FIELD));
    }
}
