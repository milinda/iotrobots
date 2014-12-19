package test;

import cgl.iotrobots.collavoid.commons.CommonUtils;
import cgl.iotrobots.collavoid.commons.Odometry_;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;
import com.rabbitmq.client.QueueingConsumer;

/**
 * Created by hjh on 12/17/14.
 */
public class Listener {
    private final static String EXCHANGE_NAME = "robot0rmq";

    public static void main(String[] argv)
            throws java.io.IOException,
            InterruptedException {

        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        Channel channel = connection.createChannel();

        channel.exchangeDeclare(EXCHANGE_NAME, "direct", true);
        System.out.println(" [*] Waiting for messages. To exit press CTRL+C");
        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, EXCHANGE_NAME, "odometryRoutingKey");

        QueueingConsumer consumer = new QueueingConsumer(channel);
        channel.basicConsume(queueName, true, consumer);

        while (true) {
            QueueingConsumer.Delivery delivery = consumer.nextDelivery();
            Odometry_ odometry_ = CommonUtils.JSONToOdometry_(delivery.getBody());

            System.out.println(" [x] Received '" + odometry_.getChildFrameId() + "'");
        }
    }

}
