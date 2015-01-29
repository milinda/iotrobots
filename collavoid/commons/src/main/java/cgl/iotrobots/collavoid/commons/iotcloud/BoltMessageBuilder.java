package cgl.iotrobots.collavoid.commons.iotcloud;

import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.RabbitMQMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class BoltMessageBuilder extends DefaultRabbitMQMessageBuilder {
    private Logger logger = LoggerFactory.getLogger(BoltMessageBuilder.class);

    @Override
    //deSerialize from rabbitmq, publisher do not need deserializer
    public List<Object> deSerialize(Object o) {
        return super.deSerialize(o);
    }

    @Override
    // serialize from storm,
    public Object serialize(Tuple tuple, Object o) {
        byte[] body = (byte[]) tuple.getValue(2);

        Object sensorId = tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
        Object time = tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD);
        Map<String, Object> props = new HashMap<String, Object>();
        props.put(Constant_storm.FIELDS.SENSOR_ID_FIELD, sensorId);
        props.put(Constant_storm.FIELDS.TIME_FIELD, time);
//        logger.info("received cmd message!");
        return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body);
    }
}
