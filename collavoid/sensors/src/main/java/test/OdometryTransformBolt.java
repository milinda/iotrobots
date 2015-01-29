package test;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Fields;
import backtype.storm.tuple.Tuple;
import backtype.storm.tuple.Values;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.Map;

public class OdometryTransformBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(OdometryTransformBolt.class);
    private OutputCollector collector;
    private Kryo kryo;
    private long index;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        kryo = Methods_RMQ.getKryo();
    }

    @Override
    public void execute(Tuple tuple) {
        if (tuple.getSourceComponent().equals(Constant_storm.Components.TIMER_COMPONENT)) {
            logger.info("time tick!!");
            collector.ack(tuple);
            return;
        }

        Odometry_ odom = (Odometry_) tuple.getValueByField(Constant_storm.FIELDS.ODOMETRY_FIELD);
        if (tuple.getValueByField("name").equals("robot0"))
            logger.info("odometry received " + index++);
        Twist_ vel = new Twist_();
        byte[] body = Methods_RMQ.serialize(kryo, vel);
        collector.emit(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Values(
                        tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD),
                        tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD),
                        body,
                        tuple.getValueByField("name"),
                        index - 1)
        );
        collector.ack(tuple);
//        index++;
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {
        outputFieldsDeclarer.declareStream(Constant_storm.Streams.VELOCITY_COMMAND_STREAM, new Fields(
                Constant_storm.FIELDS.TIME_FIELD,
                Constant_storm.FIELDS.SENSOR_ID_FIELD,
                Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD,
                "name",
                "seq"
        ));
    }
}
