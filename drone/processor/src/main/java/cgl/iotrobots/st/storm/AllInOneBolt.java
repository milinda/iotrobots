package cgl.iotrobots.st.storm;

import backtype.storm.topology.IRichBolt;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.tuple.Fields;

import java.util.Map;

public class AllInOneBolt extends ShellBoltN implements IRichBolt {
    public AllInOneBolt() {
        super("python", "DroneFrameProcessBolt.py");
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer declarer) {
        declarer.declare(new Fields(Constants.COMMAND_FIELD, Constants.TIME_FIELD));
    }

    @Override
    public Map<String, Object> getComponentConfiguration() {
        return null;
    }
}
