package test;

import cgl.iotrobots.collavoid.commons.rmqmsg.Constant_msg;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.rmqmsg.Vector3d_;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

/**
 * Created by hjh on 12/17/14.
 */
public class Talker {
    private final static String EXCHANGE_NAME = "robot0_rmq";

    public static void main(String[] argv)
            throws java.io.IOException, InterruptedException {

        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        Channel channel = connection.createChannel();

        String queueName = channel.queueDeclare().getQueue();
        channel.exchangeDeclare(EXCHANGE_NAME, "direct", true);
        String routingKey = Constant_msg.RMQ_ROUTINGKEY_PREFIX + Constant_msg.KEY_VELOCITY_CMD;
        Twist_ vel = new Twist_();
        vel.setLinear(new Vector3d_(1, 0, 0));
        int i = 0;
        while (i++ < 100) {
            channel.basicPublish(EXCHANGE_NAME, routingKey, null, vel.toJSON());
            Thread.sleep(50);
        }

        channel.close();
        connection.close();

    }
}
