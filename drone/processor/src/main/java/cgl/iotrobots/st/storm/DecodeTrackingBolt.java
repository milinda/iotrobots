package cgl.iotrobots.st.storm;

import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.tuple.Fields;

import java.util.Map;

public class DecodeTrackingBolt extends ShellBoltN implements IRichBolt {
    public DecodeTrackingBolt() {
        super("python", "DecodeTrackingBolt.py");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields("targets", "time"));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        return null;
    }
}
