package cgl.iotrobots.turtlebot.storm;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.turtlebot.commons.Compressor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Arrays;
import java.util.Map;

public class UncompressBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(UncompressBolt.class);

    private OutputCollector outputCollector;

    private Compressor compressor = new Compressor();

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        byte []data = (byte[]) tuple.getValue(0);
        try {
            int [] dist = compressor.unCompr(data);
            String sensorId = (String) tuple.getValueByField("sensorID");
            String time = (String) tuple.getValueByField("time");
            outputCollector.emit(Arrays.<Object>asList(dist, sensorId, time));
        } catch (IOException e) {
            LOG.error("Failed to uncompress the data", e);
        } finally {
            outputCollector.ack(tuple);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields("uncompress_data", "sensorID", "time"));
    }
}
