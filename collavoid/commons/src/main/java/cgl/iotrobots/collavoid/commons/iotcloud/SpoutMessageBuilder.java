package cgl.iotrobots.collavoid.commons.iotcloud;

import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.esotericsoftware.kryo.Kryo;
import com.rabbitmq.client.AMQP;
import com.ss.commons.MessageContext;
import com.ss.rabbitmq.RabbitMQMessage;

import java.util.ArrayList;
import java.util.List;

public class SpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
    private Kryo kryo;

    public SpoutMessageBuilder() {
        super();
        kryo = Methods_RMQ.getKryo();
    }

    @Override
    public List<Object> deSerialize(Object o) {
        List<Object> objects = deSerialize_(o);
        List<Object> res = new ArrayList<Object>();

        // here reverse the sequence of the fields
        // idx
        Object idx = null;
        if (objects.size() > 3)
            idx = objects.remove(3);
        //time, be careful about time synchronization in cluster
        Object next = objects.remove(2);
        res.add(Long.parseLong(next.toString()));
        //id
        next = objects.remove(1);
        res.add(next.toString());
        //data
        byte[] body = (byte[]) objects.remove(0);
        Object obj = Methods_RMQ.deSerialize(kryo, body);
        res.add(obj);
        //idx
        if (idx != null)
            res.add(Long.parseLong(idx.toString()));
        return res;
    }

    public List<Object> deSerialize_(Object o) {
        ArrayList tuples = new ArrayList();
        if (o instanceof MessageContext && ((MessageContext) o).getMessage() instanceof RabbitMQMessage) {
            RabbitMQMessage rabbitMQMessage = (RabbitMQMessage) ((MessageContext) o).getMessage();
            AMQP.BasicProperties properties = rabbitMQMessage.getProperties();
            Object time = null;
            Object sensorId = null;
            Object idx = null;
            if (properties != null && properties.getHeaders() != null) {
                sensorId = properties.getHeaders().get("sensorID");
                time = properties.getHeaders().get("time");
                idx = properties.getHeaders().get("agentIndex");
            }

            tuples.add(rabbitMQMessage.getBody());
            if (sensorId != null) {
                tuples.add(sensorId.toString());
            }

            if (time != null) {
                tuples.add(time.toString());
            } else {
                tuples.add(Long.toString(System.currentTimeMillis()));
            }

            if (idx != null) {
                tuples.add(idx.toString());
            }
        }

        return tuples;
    }

    @Override
    public Object serialize(Tuple tuple, Object o) {
        return super.serialize(tuple, o);
    }


}
