package cgl.iotrobots.slam.streaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.slam.core.app.MapUpdater;

import java.util.Map;

public class MapBuildingBolt extends BaseRichBolt {
    private MapUpdater mapUpdater;

    private OutputCollector outputCollector;

    private TopologyContext topologyContext;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
        this.topologyContext = topologyContext;
    }

    @Override
    public void execute(Tuple tuple) {

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
