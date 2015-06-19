package cgl.iotrobots.collavoid.topology;

import backtype.storm.spout.SpoutOutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichSpout;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Values;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;

import java.util.Map;

public class TimerSpout extends BaseRichSpout {
    SpoutOutputCollector collector;
    int cnt = 0;

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.CTLPUB_TIMEER_STREAM,
                new Fields(Constant_storm.FIELDS.TIMER_TICK_FIELD));
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.TIMEOUT_STREAM,
                new Fields(Constant_storm.FIELDS.TIMER_TICK_FIELD));
    }

    @Override
    public void open(Map map, TopologyContext topologyContext, SpoutOutputCollector spoutOutputCollector) {
        collector = spoutOutputCollector;
    }

    @Override
    public void nextTuple() {
        cnt++;
        Utils.sleep(10);
        collector.emit(Constant_storm.Streams.CTLPUB_TIMEER_STREAM,
                new Values(System.currentTimeMillis()));

        if (cnt > 300) {
            collector.emit(Constant_storm.Streams.TIMEOUT_STREAM,
                    new Values(System.currentTimeMillis()));
            cnt = 0;
        }
    }
}
