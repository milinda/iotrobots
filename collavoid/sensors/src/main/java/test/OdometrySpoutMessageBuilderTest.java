package test;

import backtype.storm.tuple.Tuple;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.esotericsoftware.kryo.Kryo;
import com.rabbitmq.client.AMQP;
import com.ss.commons.MessageContext;
import com.ss.rabbitmq.RabbitMQMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class OdometrySpoutMessageBuilderTest extends DefaultRabbitMQMessageBuilder {

    private Logger logger = LoggerFactory.getLogger(OdometrySpoutMessageBuilderTest.class);
    private long index;

    private Kryo kryo = Methods_RMQ.getKryo();

    @Override
    //deSerialize from rabbitmq
    public List<Object> deSerialize(Object o) {
        List<Object> objects = super.deSerialize(o);
        RabbitMQMessage rabbitMQMessage = (RabbitMQMessage) ((MessageContext) o).getMessage();
        AMQP.BasicProperties properties = rabbitMQMessage.getProperties();
        String robotname = properties.getHeaders().get("name").toString();
        index = Long.parseLong(properties.getHeaders().get("seq").toString());
        List<Object> res = new ArrayList<>();
        // here reverse the sequence of the fields
        //time
        Object next = objects.remove(2);
        res.add(next.toString());
//        System.out.println("++++++++++++++++++++++++++++odom time: " + res.get(0).toString());
        //id
        next = objects.remove(1);
        res.add(next.toString());
//        System.out.println("++++++++++++++++++++++++++++odom id: " + res.get(1).toString());
        //odometry
        byte[] body = (byte[]) objects.remove(0);
        if (robotname.equals("robot0")) {
            logger.info("odomtery received {}", index);

        }
        Object odom = Methods_RMQ.deSerialize(kryo, body, Odometry_.class);
//            Object odom = Methods_RMQ.deSerialize(body, Odometry_.class);
        if (!(odom instanceof Odometry_))
            System.out.println("not a instance of Odometry_");
        res.add(odom);
        res.add(robotname);
        return res;
    }


    @Override
    // serialize from storm, spout do not need serializer
    public Object serialize(Tuple tuple, Object o) {
        return super.serialize(tuple, o);
    }
}
