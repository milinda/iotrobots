package cgl.iotrobots.st.storm;

import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.tuple.Fields;

import java.util.Map;

public class PlanningBolt extends ShellBoltN implements IRichBolt {
    public PlanningBolt() {
        super("python", "PlanningBolt.py");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields("command", "time"));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        return null;
    }
}
