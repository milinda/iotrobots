package cgl.iotrobots.collavoid.topology;

import backtype.storm.spout.SpoutOutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichSpout;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.planners.Parameters;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.Map;

public class TimerBolt extends BaseRichSpout {
    private SpoutOutputCollector collector;
    private long pubMePeriod = (long) (1 / Parameters.PUBLISH_ME_FREQUENCY * 1000);
    private long controlPeriod = (long) (1 / Parameters.CONTROLLER_FREQUENCY * 1000);
    private long lastMepub;
    private long lastControlled;
    private long pubseq;
    private long controlseq;

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declareStream(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM,
                new Fields(Constant_storm.FIELDS.TIMER_FIELD));
        declarer.declareStream(Constant_storm.Streams.CONTROLLER_TIMER_STREAM,
                new Fields(Constant_storm.FIELDS.TIMER_FIELD));
    }

    @Override
    public void open(Map conf, TopologyContext context, SpoutOutputCollector collector) {
        this.collector = collector;
        lastMepub = System.currentTimeMillis();
        lastControlled = System.currentTimeMillis();
        pubseq = 0;
        controlseq = 0;
    }

    @Override
    public void nextTuple() {
        Utils.sleep(10);
        if (System.currentTimeMillis() - lastMepub > pubMePeriod)
            collector.emit(Constant_storm.Streams.PUBLISH_ME_TIMER_STREAM, new Values(pubseq++));
        if (System.currentTimeMillis() - lastControlled > controlPeriod)
            collector.emit(Constant_storm.Streams.CONTROLLER_TIMER_STREAM, new Values(controlseq++));
    }

    @Override
    public void ack(Object msgId) {
        super.ack(msgId);
    }

    @Override
    public void fail(Object msgId) {
        super.fail(msgId);
    }
}
