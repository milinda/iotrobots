package cgl.iotrobots.st.storm;

import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.tuple.Fields;

import java.util.Map;

public class TrackingBolt extends ShellBoltN implements IRichBolt {
    public TrackingBolt() {
        super("python", "DroneFrameProcessBolt.py");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields("image", "time"));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        return null;
    }
}
