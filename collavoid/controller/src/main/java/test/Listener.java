package test;

import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Odometry_;
import com.rabbitmq.client.*;

import java.io.IOException;

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
        final Channel channel = connection.createChannel();

        channel.exchangeDeclare(EXCHANGE_NAME, "direct", true);
        System.out.println(" [*] Waiting for messages. To exit press CTRL+C");
        String queueName = channel.queueDeclare().getQueue();
        channel.queueBind(queueName, EXCHANGE_NAME, "RoutingKey_Odometry");


        try {
            channel.basicConsume(queueName, false, "odometry" + "Tag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Odometry_ odometry_ = (Odometry_) Methods_RMQ.deSerialize(body, Odometry_.class);
                            System.out.println(odometry_);
                            channel.basicAck(deliveryTag, false);
                        }
                    });
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }
//
//        QueueingConsumer consumer = new QueueingConsumer(channel);
//        channel.basicConsume(queueName, true, consumer);
//
//        while (true) {
//            QueueingConsumer.Delivery delivery = consumer.nextDelivery();
//            Odometry_ odometry_ = JsonConverter.JSONToOdometry_(delivery.getBody());
//
//            System.out.println(" [x] Received '" + odometry_.getChildFrameId() + "'");
//        }
    }

}
