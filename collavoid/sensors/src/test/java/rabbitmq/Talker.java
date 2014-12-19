package rabbitmq;

import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

/**
 * Created by hjh on 12/17/14.
 */
public class Talker {
    private final static String EXCHANGE_NAME = "robot0rmq";

    public static void main(String[] argv)
            throws java.io.IOException, InterruptedException {

        ConnectionFactory factory = new ConnectionFactory();
        factory.setHost("localhost");
        Connection connection = factory.newConnection();
        Channel channel = connection.createChannel();

        // not needed, only need routing key
        String queueName = channel.queueDeclare().getQueue();
        channel.exchangeDeclare(EXCHANGE_NAME, "direct", true);

        String message = "this is a simple talker.";
        int i = 0;
        while (i++ < 100) {
            channel.basicPublish(EXCHANGE_NAME, "routingkey", null, message.getBytes());
            Thread.sleep(50);
        }

        channel.close();
        connection.close();

    }

}
