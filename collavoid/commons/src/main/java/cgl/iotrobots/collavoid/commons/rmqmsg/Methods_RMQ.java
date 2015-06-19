package cgl.iotrobots.collavoid.commons.rmqmsg;

import cgl.iotrobots.collavoid.commons.planners.*;
import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;
import com.rabbitmq.client.Address;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.util.concurrent.ExecutorService;

public class Methods_RMQ {

    public static Connection getConnection(Address[] addresses,
                                           String url,
                                           ExecutorService executorService) {
        Connection connection;
        try {
            ConnectionFactory factory = new ConnectionFactory();
            if (addresses == null) {
                factory.setUri(url);
                if (executorService != null) {
                    connection = factory.newConnection(executorService);
                } else {
                    connection = factory.newConnection();
                }
            } else {
                if (executorService != null) {
                    connection = factory.newConnection(executorService, addresses);
                } else {
                    connection = factory.newConnection(addresses);
                }
            }
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

        return connection;

    }


    public static Kryo getKryo() {
        Kryo res = new Kryo();
        registerSerialization(res);
        return res;
    }

    public static void registerSerialization(Kryo kryo) {
        kryo.register(Vector3d_.class, 101);
        kryo.register(Vector4d_.class, 102);
        kryo.register(Vector2.class, 103);
        kryo.register(Header_.class, 104);
        kryo.register(Twist_.class, 105);
        kryo.register(Pose_.class, 106);
        kryo.register(PoseShareMsg_.class, 107);
        kryo.register(PoseStamped_.class, 108);
        kryo.register(BasicConfig_.class, 109);
        kryo.register(PointCloud2_.class, 110);
        kryo.register(Odometry_.class, 111);
        kryo.register(PoseArray_.class, 112);
        kryo.register(LaserScan_.class, 120);

        //for topology
        kryo.register(AgentState.class, 122);
        kryo.register(AgentCtlPubState.class, 123);

        kryo.register(Position.class, 113);
        kryo.register(Agent.class, 114);
        kryo.register(Neighbor.class, 115);
        kryo.register(VO.class, 116);
        kryo.register(Line.class, 117);
        kryo.register(LinePair.class, 118);
        kryo.register(Obstacle.class, 119);
    }

    public static byte[] serialize(Kryo kryo, Object object) {
        ByteArrayOutputStream byteArrayOutputStream = new ByteArrayOutputStream();
        Output output = new Output(byteArrayOutputStream);
        kryo.writeClassAndObject(output, object);
        output.close();
        return byteArrayOutputStream.toByteArray();
    }

    public static Object deSerialize(Kryo kryo, byte[] b) {
        return kryo.readClassAndObject(new Input(new ByteArrayInputStream(b)));
    }


}
