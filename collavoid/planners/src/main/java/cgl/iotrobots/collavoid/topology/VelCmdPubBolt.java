package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rabbitmq.Message;
import cgl.iotrobots.collavoid.commons.rabbitmq.RabbitMQSender;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import com.rabbitmq.client.ConnectionFactory;


import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class VelCmdPubBolt extends BaseRichBolt {
    private String sensorID = null;
    private Message msg;
    private OutputCollector collector;
    private RabbitMQSender msgSender;
    private String url = "amqp://localhost:5672";
    private String routingKey;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        collector = outputCollector;
        routingKey = Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_VELOCITY_CMD;
    }

    @Override
    public void execute(Tuple tuple) {
        if (sensorID == null) {
            sensorID = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            String exchange = sensorID + Constant_msg.RMQ_EXCHANGE_SUFFIX;
            msgSender = new RabbitMQSender(url, exchange);
            try {
                msgSender.open(Constant_msg.TYPE_EXCHANGE_TOPIC);
            } catch (Exception e) {
                e.printStackTrace();
            }

        }
        Object input = tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
        if (!(input instanceof Twist_)) {
            throw new IllegalArgumentException("Not a valid velocity command!");
        }
        Twist_ velCmd = (Twist_) input;
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
