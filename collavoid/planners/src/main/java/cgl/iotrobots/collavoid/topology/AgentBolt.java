package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.planners.Agent;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;


import java.util.Map;

public class AgentBolt extends BaseRichBolt {
    private Agent agent;

    @Override
    public void execute(Tuple tuple) {
        if (tuple.getSourceComponent().equals(Constant_storm.Components.SCAN_COMPONENT)) {

        }
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        agent = new Agent("agent");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
