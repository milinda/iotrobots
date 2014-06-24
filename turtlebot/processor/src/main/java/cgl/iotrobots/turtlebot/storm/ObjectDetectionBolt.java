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

    private int count = 0;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        this.outputCollector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        System.out.println("Got message and sending Motion command");
        if (count % 2 == 1) {
            outputCollector.emit(Arrays.<Object>asList(new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, 1))));
        } else {
            outputCollector.emit(Arrays.<Object>asList(new Motion(new Velocity(0, 0, 0), new Velocity(0, 0, -1))));
        }
        count++;

    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declare(new Fields("control"));
    }
}
