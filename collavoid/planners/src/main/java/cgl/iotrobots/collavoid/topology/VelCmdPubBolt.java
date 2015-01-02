package cgl.iotrobots.collavoid.topology;

import backtype.storm.task.OutputCollector;
import backtype.storm.task.TopologyContext;
import backtype.storm.topology.BasicOutputCollector;
import backtype.storm.topology.OutputFieldsDeclarer;
import backtype.storm.topology.base.BaseBasicBolt;
import backtype.storm.topology.base.BaseRichBolt;
import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.iotrobots.collavoid.commons.storm.Methods_storm;
import com.rabbitmq.client.ConnectionFactory;
import io.latent.storm.rabbitmq.Message;
import io.latent.storm.rabbitmq.RabbitMQProducer;
import io.latent.storm.rabbitmq.config.ConnectionConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfig;
import io.latent.storm.rabbitmq.config.ConsumerConfigBuilder;

import java.io.IOException;
import java.util.Map;

public class VelCmdPubBolt extends BaseRichBolt {
    private RabbitMQProducer cmdSender;
    private ConnectionConfig connectionConfig;
    private ConsumerConfig consumerConfig;
    private String sensorID = null;
    private Message msg;
    private OutputCollector collector;

    @Override
    public void prepare(Map map, TopologyContext topologyContext, OutputCollector outputCollector) {
        connectionConfig = new ConnectionConfig(
                "localhost",
                5672,
                "guest",
                "guest",
                ConnectionFactory.DEFAULT_VHOST,
                10); // host, port, username, password, virtualHost, heartBeat
        consumerConfig = new ConsumerConfigBuilder().connection(connectionConfig)
                .queue("")
                .prefetch(200)
                .requeueOnFail()
                .build();
        collector = outputCollector;
    }

    @Override
    public void execute(Tuple tuple) {
        if (sensorID == null) {
            sensorID = (String) tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            cmdSender = new RabbitMQProducer(new Methods_storm.StormDeclarator(
                    sensorID + Constant_msg.RMQ_EXCHANGE_SUFFIX,
                    Constant_msg.RMQ_QUEUE_PREFIX + Constant_msg.KEY_VELOCITY_CMD,
                    Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_VELOCITY_CMD,
                    ""));
            cmdSender.open(consumerConfig.asMap());
        }
        Object input = tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
        if (!(input instanceof Twist_)) {
            throw new IllegalArgumentException("Not a valid velocity command!");
        }
        Twist_ velCmd = (Twist_) input;
        try {
            msg = new Message(velCmd.toJSON());
        } catch (IOException e) {
            e.printStackTrace();
        }
        cmdSender.send(msg);
        collector.ack(tuple);
    }

    @Override
    public void declareOutputFields(OutputFieldsDeclarer outputFieldsDeclarer) {

    }
}
