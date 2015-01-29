package cgl.iotrobots.collavoid.commons.iotcloud;

import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.esotericsoftware.kryo.Kryo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class SpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
    private Kryo kryo;
    private Logger logger = LoggerFactory.getLogger(SpoutMessageBuilder.class);

    public SpoutMessageBuilder() {
        super();
        kryo = Methods_RMQ.getKryo();
    }

    @Override
    public List<Object> deSerialize(Object o) {
//        logger.info("received message from channel");
        List<Object> objects = super.deSerialize(o);
        List<Object> res = new ArrayList<>();

        // here reverse the sequence of the fields
        //time
        Object next = objects.remove(2);
        res.add(Long.parseLong(next.toString()));
        //id
        next = objects.remove(1);
        res.add(next.toString());
        //odometry
        byte[] body = (byte[]) objects.remove(0);
        Object obj = Methods_RMQ.deSerialize(kryo, body);

        res.add(obj);
        return res;
    }

    @Override
    public Object serialize(Tuple tuple, Object o) {
        return super.serialize(tuple, o);
    }


}
