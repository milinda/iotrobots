package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.rabbitmq.client.Address;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

import java.io.IOException;
import java.util.concurrent.ExecutorService;

/**
 * Created by hjh on 12/21/14.
 */
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


        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

        return connection;

    }

    public static Channel getChannel(Address[] addresses,
                                     String url,
                                     ExecutorService executorService,
                                     String exchangeName,
                                     String exchangeType) {
        Connection connection;
        Channel channel;
        boolean durable = true;
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
            channel = connection.createChannel();
            if (exchangeName != null) {
                channel.exchangeDeclare(exchangeName, exchangeType, durable);
            }


        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        } catch (Exception e) {
            String msg = "Error connecting to broker";
            throw new RuntimeException(msg, e);
        }

        return channel;

    }

    public static void publishMsg(Channel channel, RMQContext context, byte[] body) {

        try {
            channel.basicPublish(
                    context.EXCHANGE_NAME,
                    context.ROUTING_KEY,
                    null,
                    body);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

}
