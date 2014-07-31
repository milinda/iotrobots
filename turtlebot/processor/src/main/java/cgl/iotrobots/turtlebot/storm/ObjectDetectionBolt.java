package cgl.iotrobots.turtlebot.storm;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.turtlebot.commons.Compressor;
import cgl.iotrobots.turtlebot.commons.Motion;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Arrays;
import java.util.Map;

public class ObjectDetectionBolt extends BaseRichBolt {
    private static Logger LOG = LoggerFactory.getLogger(ObjectDetectionBolt.class);

    private OutputCollector outputCollector;

    private PositionCalculator positionCalculator;

    private boolean compressed;

    public ObjectDetectionBolt(boolean compressed) {
        this.compressed = compressed;
    }

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
        this.positionCalculator = new PositionCalculator();
    }

    @Override
    public void execute(Tuple tuple) {
        LOG.info("Got message and sending Motion command");
        int[] dist;
        try {
            Compressor compressor = new Compressor();
            if (compressed) {
                byte []data = (byte[]) tuple.getValue(0);
                dist = compressor.unCompr(data);
            } else {
                dist = (int []) tuple.getValue(0);
            }

            Motion motion = positionCalculator.calculatePosition(dist);
            String sensorId = (String) tuple.getValueByField("sensorID");
            String time = (String) tuple.getValueByField("time");
            outputCollector.emit(Arrays.<Object>asList(motion, sensorId, time));
        } catch (IOException e) {
            LOG.error("Failed to uncomress the data", e);
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields("control", "sensorID", "time"));
    }
}
