package rabbitmq;

import com.rabbitmq.client.*;

import java.io.IOException;

public class Listener {
    private final static String EXCHANGE_NAME = "robot0rmq";
    private static boolean autoAck = false;
    private static Channel channel;

    public static void main(String[] argv)
            throws java.io.IOException,
            InterruptedException {

        // first connection
        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        channel = connection.createChannel();

        // then channel
        channel.exchangeDeclare(EXCHANGE_NAME, "direct", true);
        System.out.println(" [*] Waiting for messages. To exit press CTRL+C");
        // for listening node need to know queue name
        String queueName = channel.queueDeclare().getQueue();
        // bind queue and routing key
        channel.queueBind(queueName, EXCHANGE_NAME, "odometryRoutingKey");

        // conventional way of processing queue
        QueueingConsumer consumer = new QueueingConsumer(channel);
        channel.basicConsume(queueName, true, consumer);

        int i = 0;
        while (i++ < 100) {
            QueueingConsumer.Delivery delivery = consumer.nextDelivery();
            String message = delivery.getBody().toString();

            System.out.println(" [x] Received '" + message + "'");
        }

        // an easier way
        try {
            channel.basicConsume(queueName, autoAck, "WorkerTag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            String msg = new String(body);
                            System.out.println("Received: " + msg);
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

    }
}
