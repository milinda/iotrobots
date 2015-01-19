package cgl.iotrobots.collavoid.commons.rmqmsg;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.rabbitmq.client.Address;
import com.rabbitmq.client.Channel;
import com.rabbitmq.client.Connection;
import com.rabbitmq.client.ConnectionFactory;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ExecutorService;

/**
 * Created by hjh on 12/21/14.
 */
public class Methods_RMQ {

    public static void clearQueues(Map<String, RMQContext> RMQContexts) {
        for (Map.Entry<String, RMQContext> context : RMQContexts.entrySet()) {
            if (context.getValue().CHANNEL == null || !context.getKey().equals(Constant_msg.KEY_VELOCITY_CMD))
                continue;
            try {
                context.getValue().CHANNEL.queuePurge(context.getValue().QUEUE_NAME);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

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


    public static Channel getChannel(Address[] addresses,
                                     String url,
                                     ExecutorService executorService) {
        Channel channel;
        try {
            channel = getConnection(addresses, url, executorService).createChannel();
        } catch (IOException e) {
            String msg = "Error consuming the message";
            throw new RuntimeException(msg, e);
        }
        return channel;
    }

    public static void publishMsg(RMQContext context, byte[] body) {
        if (context.CHANNEL.isOpen()) {
        try {
            context.CHANNEL.basicPublish(
                    context.EXCHANGE_NAME,
                    context.ROUTING_KEY,
                    null,
                    body);

        } catch (IOException e) {
            e.printStackTrace();
        }
        }

    }

    public static byte[] serialize(Object value) {
        ObjectMapper mapper = new ObjectMapper();
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        try {
            mapper.writeValue(outputStream, value);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return outputStream.toByteArray();
    }

    public static Object deserialize(byte[] data, Class c) {
        Object res = new Object();
        ObjectMapper mapper = new ObjectMapper(); // can reuse, share globally
        mapper.setVisibility(PropertyAccessor.FIELD, JsonAutoDetect.Visibility.ANY);
        try {
            res = mapper.readValue(data, c);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return res;
    }

}
