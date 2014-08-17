package cgl.iotrobots.st.storm;

import backtype.storm.tuple.Tuple;
import cgl.iotcloud.core.transport.TransportConstants;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.RabbitMQMessage;
import org.apache.commons.codec.binary.Base64;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class CustomMessageBuilder extends DefaultRabbitMQMessageBuilder {
    @Override
    public List<Object> deSerialize(Object o) {
        List<Object> objects = super.deSerialize(o);

        byte []body = (byte[]) objects.remove(0);
        objects.add(0, new String(Base64.encodeBase64(body)));
        Object on = objects.remove(1);
        objects.add(1, on.toString());

        on = objects.remove(2);
        objects.add(2, on.toString());
        return objects;
    }

    @Override
    public Object serialize(Tuple tuple, Object o) {
        String body = (String) tuple.getValueByField("body");
        Object sensorId = tuple.getValueByField(TransportConstants.SENSOR_ID);
        Object time = tuple.getValueByField("time");
        Map<String, Object> props = new HashMap<String, Object>();
        props.put(TransportConstants.SENSOR_ID, sensorId);
        props.put("time", time);
        // System.out.println("Sending message" + motion);
        return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body.getBytes());
    }
}
