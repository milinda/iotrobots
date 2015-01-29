package test;

import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.esotericsoftware.kryo.Kryo;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.RabbitMQMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VelocityPublisherBoltMessageBuilder extends DefaultRabbitMQMessageBuilder {
    private Kryo kryo = Methods_RMQ.getKryo();
    private Logger logger = LoggerFactory.getLogger(VelocityPublisherBoltMessageBuilder.class);
    private long index;

    @Override
    //deSerialize from rabbitmq, publisher do not need deserializer
    public List<Object> deSerialize(Object o) {
        return super.deSerialize(o);
    }

    @Override
    // serialize from storm,
    public Object serialize(Tuple tuple, Object o) {
        byte[] body = (byte[]) tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
//            byte[] body = Methods_RMQ.serialize(tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD));
        Object sensorId = tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        Object time = tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD);
        index = Long.parseLong(tuple.getValueByField("seq").toString());
        String robotname = tuple.getStringByField("name");
        Map<String, Object> props = new HashMap<String, Object>();
        props.put(Constant_storm.FIELDS.SENSOR_ID_FIELD, sensorId);
        props.put("name", robotname);
        props.put(Constant_storm.FIELDS.TIME_FIELD, time);
        if (robotname.equals("robot0")) {
            logger.info("Send twist_ message {}", index);
//            index++;
        }
        return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body);
    }
}