package cgl.iotrobots.collavoid.commons.iotcloud;

import backtype.storm.tuple.Tuple;
import backtype.storm.utils.Utils;
import cgl.iotrobots.collavoid.commons.rmqmsg.*;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.sensorstream.core.rabbitmq.DefaultRabbitMQMessageBuilder;
import com.esotericsoftware.kryo.Kryo;
import com.rabbitmq.client.AMQP;
import com.ss.rabbitmq.RabbitMQMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MessageBuilders {
    private static Logger logger = LoggerFactory.getLogger(MessageBuilders.class);

    //spouts
    public static class OdometrySpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
        private Kryo kryo = Methods_RMQ.getKryo();
        @Override
        //deSerialize from rabbitmq
        public List<Object> deSerialize(Object o) {
            List<Object> objects = super.deSerialize(o);
            List<Object> res = new ArrayList<>();
            // here reverse the sequence of the fields
            //time
            Object next = objects.remove(2);
            res.add(next.toString());
//            System.out.println("odom time: " + res.get(0).toString());
            //id
            next = objects.remove(1);
            res.add(next.toString());
//            System.out.println("odom id: " + res.get(1).toString());
            //odometry
            byte[] body = (byte[]) objects.remove(0);
            Object odom = Methods_RMQ.deSerialize(kryo, body, Odometry_.class);
//            Object odom = Methods_RMQ.deSerialize(body, Odometry_.class);
            if (!(odom instanceof Odometry_))
                logger.error("Not an instance of Odometry_");
            res.add(odom);
            return res;
        }

        @Override
        // serialize from storm, spout do not need serializer
        public Object serialize(Tuple tuple, Object o) {
            return super.serialize(tuple, o);
        }
    }

    public static class ScanSpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
        private Kryo kryo = Methods_RMQ.getKryo();
        @Override
        //deSerialize from rabbitmq
        public List<Object> deSerialize(Object o) {
            List<Object> objects = super.deSerialize(o);
            List<Object> res = new ArrayList<>();
            // here reverse the sequence of the fields
            //time
            Object next = objects.remove(2);
            res.add(next.toString());
            System.out.println("scan time: " + res.get(0).toString());
            //id
            next = objects.remove(1);
            res.add(next.toString());
            System.out.println("scan id: " + res.get(1).toString());
            //scan
            byte[] body = (byte[]) objects.remove(0);
            res.add(Methods_RMQ.deSerialize(kryo, body, PointCloud2_.class));
//            res.add(Methods_RMQ.deSerialize( body, PointCloud2_.class));
            return res;
        }

        @Override
        // serialize from storm, spout do not need serializ = new Kryo()er
        public Object serialize(Tuple tuple, Object o) {
            return super.serialize(tuple, o);
        }
    }

    public static class PoseArraySpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
        private Kryo kryo = Methods_RMQ.getKryo();
        @Override
        //deSerialize from rabbitmq
        public List<Object> deSerialize(Object o) {
            List<Object> objects = super.deSerialize(o);
            List<Object> res = new ArrayList<>();
            // here reverse the sequence of the fields
            //time
            Object next = objects.remove(2);
            res.add(next.toString());
            //id
            next = objects.remove(1);
            res.add(next.toString());
            //pose array
            byte[] body = (byte[]) objects.remove(0);
            res.add(Methods_RMQ.deSerialize(kryo, body, PoseArray_.class));
//            res.add(Methods_RMQ.deSerialize( body, PoseArray_.class));
            return res;
        }

        @Override
        // serialize from storm, odometry is spout do not need serializer
        public Object serialize(Tuple tuple, Object o) {
            return super.serialize(tuple, o);
        }
    }

    public static class BaseConfigSpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
        private Kryo kryo = Methods_RMQ.getKryo();
        @Override
        //deSerialize from rabbitmq
        public List<Object> deSerialize(Object o) {
            List<Object> objects = super.deSerialize(o);
            List<Object> res = new ArrayList<>();
            // here reverse the sequence of the fields
            //time
            Object next = objects.remove(2);
            res.add(next.toString());
            //id
            next = objects.remove(1);
            res.add(next.toString());
            //baseconfig
            byte[] body = (byte[]) objects.remove(0);
            res.add(Methods_RMQ.deSerialize(kryo, body, BaseConfig_.class));
//            res.add(Methods_RMQ.deSerialize( body, BaseConfig_.class));
//            objects.add(Utils.deserialize(body));
            return res;
        }

        @Override
        // serialize from storm, odometry is spout do not need serializer
        public Object serialize(Tuple tuple, Object o) {
            return super.serialize(tuple, o);
        }
    }

    public static class PoseShareOutSpoutMessageBuilder extends DefaultRabbitMQMessageBuilder {
        private Kryo kryo = Methods_RMQ.getKryo();
        @Override
        //deSerialize from rabbitmq
        public List<Object> deSerialize(Object o) {
            List<Object> objects = super.deSerialize(o);
            List<Object> res = new ArrayList<>();
            // here reverse the sequence of the fields
            //time
            Object next = objects.remove(2);
            res.add(next.toString());
            //id
            next = objects.remove(1);
            res.add(next.toString());
            //baseconfig
            byte[] body = (byte[]) objects.remove(0);
            res.add(Methods_RMQ.deSerialize(kryo, body, PoseShareMsg_.class));
//            res.add(Methods_RMQ.deSerialize( body, PoseShareMsg_.class));
            return res;
        }

        @Override
        // serialize from storm, odometry is spout do not need serializer
        public Object serialize(Tuple tuple, Object o) {
            return super.serialize(tuple, o);
        }
    }

    //bolts
    public static class VelocityPublisherBoltMessageBuilder extends DefaultRabbitMQMessageBuilder {
        @Override
        //deSerialize from rabbitmq, publisher do not need deserializer
        public List<Object> deSerialize(Object o) {
            return super.deSerialize(o);
        }

        @Override
        // serialize from storm,
        public Object serialize(Tuple tuple, Object o) {
            byte[] body = (byte[]) tuple.getValueByField(Constant_storm.FIELDS.VELOCITY_COMMAND_FIELD);
            Object sensorId = tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            Object time = tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put(Constant_storm.FIELDS.SENSOR_ID_FIELD, sensorId);
            props.put(Constant_storm.FIELDS.TIME_FIELD, time);
            return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body);
        }
    }

    public static class PoseShareInBoltMessageBuilder extends DefaultRabbitMQMessageBuilder {
        @Override
        //deSerialize from rabbitmq, publisher do not need deserializer
        public List<Object> deSerialize(Object o) {
            return super.deSerialize(o);
        }

        @Override
        // serialize from storm,
        public Object serialize(Tuple tuple, Object o) {
            byte[] body = (byte[]) tuple.getValueByField(Constant_storm.FIELDS.POSE_SHARE_FIELD);

            Object sensorId = tuple.getValueByField(Constant_storm.FIELDS.SENSOR_ID_FIELD);
            Object time = tuple.getValueByField(Constant_storm.FIELDS.TIME_FIELD);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put(Constant_storm.FIELDS.SENSOR_ID_FIELD, sensorId);
            props.put(Constant_storm.FIELDS.TIME_FIELD, time);
            return new RabbitMQMessage(null, null, null, new AMQP.BasicProperties.Builder().headers(props).build(), body);
        }
    }
}
