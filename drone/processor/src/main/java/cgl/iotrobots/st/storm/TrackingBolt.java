package cgl.iotrobots.st.storm;

import backtype.storm.task.ShellBolt;
import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.tuple.Fields;

import java.util.Map;

public class TrackingBolt extends ShellBolt implements IRichBolt {
    public TrackingBolt() {
        super("python", "TrackingBolt.py");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields("targets", "sensorID", "time"));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        return null;
    }
}
