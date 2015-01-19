package cgl.iotrobots.collavoid.topologyStreaming;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rabbitmq.Message;
import cgl.iotrobots.collavoid.commons.rabbitmq.RabbitMQSender;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class VelCmdPubBolt extends BaseRichBolt {
    private Logger logger = LoggerFactory.getLogger(VelCmdPubBolt.class);
    private String sensorID = null;
    private Message msg;
    private OutputCollector collector;
    private RabbitMQSender msgSender;
    private String routingKey;
    private int init = 0;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        msgSender = new RabbitMQSender(Constant_msg.RMQ_URL, Constant_msg.KEY_VELOCITY_CMD);
        try {
            msgSender.open(Constant_msg.TYPE_EXCHANGE_TOPIC);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void execute(Tuple tuple) {
//        if (init++ < 100) {
//            collector.ack(tuple);
//            return;
//        }
//        System.out.println("Debug-ncmd in: "+System.currentTimeMillis());
        sensorID = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        routingKey = new RMQContext(Constant_msg.KEY_VELOCITY_CMD, sensorID).ROUTING_KEY;
        Object input = tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
        if (!(input instanceof Twist_)) {
            throw new IllegalArgumentException("Not a valid velocity command!");
        }
        Twist_ velCmd = (Twist_) input;
        velCmd.setTime(tuple.getLongByField(Constant_storm.FIELDS.TIME_FIELD));
        try {
            msg = new Message(Methods_RMQ.serialize(velCmd), new HashMap<String, Object>());
            msgSender.send(msg, routingKey);
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

        collector.ack(tuple);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
