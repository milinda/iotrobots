package cgl.iotrobots.turtlebot.storm;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.turtlebot.commons.Motion;
import cgl.iotrobots.turtlebot.commons.Velocity;

import java.util.Arrays;
import java.util.Map;

public class ObjectDetectionBolt extends BaseRichBolt {
    private OutputCollector outputCollector;

    private ObjectDetector objectDetector;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
        this.objectDetector = new ObjectDetector();
    }

    @Override
    public void execute(Tuple tuple) {
        System.out.println("Got message and sending Motion command");

        boolean detect = objectDetector.detect((byte[]) tuple.getValue(0));

        if (detect) {
            System.out.println("detected object");
            outputCollector.emit(Arrays.<Object>asList(new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 0))));
        } else {
            System.out.println("nnnnnnnnnnnnnnnnnnnnnnoooooooooooooooooooooooooo");
        }
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields("control"));
    }
}
