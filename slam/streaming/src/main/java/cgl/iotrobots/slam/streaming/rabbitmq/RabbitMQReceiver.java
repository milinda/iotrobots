package cgl.iotrobots.slam.streaming.rabbitmq;

import com.rabbitmq.client.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RabbitMQReceiver {
    private static Logger LOG = LoggerFactory.getLogger(RabbitMQReceiver.class);

    private String exchangeName;
    private String url;
    private Connection connection;
    private Channel channel;
    private Map<String, QueueDetails> queueDetailsMap = new HashMap<String, QueueDetails>();

    public RabbitMQReceiver(String brokerUrl, String exchangeName) throws Exception {
        this.exchangeName = exchangeName;
        this.url = brokerUrl;

        try {
            connection = createConnection();
            channel = connection.createChannel();

            channel.exchangeDeclare(exchangeName, "direct", false);
        } catch (Exception e) {
            String msg = "could not open channel for exchange " + exchangeName;
            LOG.error(msg);
            throw new Exception(msg, e);
        }
    }

    public String listen(final MessageHandler handler) throws Exception {
        try {
            Map<String, String> props = handler.getProperties();
            final String routingKey = props.get(MessagingConstants.RABBIT_ROUTING_KEY);
            if (routingKey == null) {
                throw new IllegalArgumentException("The routing key must be present");
            }

            String queueName = props.get(MessagingConstants.RABBIT_QUEUE);
            String consumerTag = props.get(MessagingConstants.RABBIT_CONSUMER_TAG);
            if (queueName == null) {
                queueName = channel.queueDeclare().getQueue();
            } else {
                channel.queueDeclare(queueName, true, false, false, null);
            }

            if (consumerTag == null) {
                consumerTag = "default";
            }

            String id = routingKey + "." + queueName;
            channel.queueBind(queueName, exchangeName, routingKey);
            channel.basicConsume(queueName, true, consumerTag, new DefaultConsumer(channel) {
                @Override
                public void handleDelivery(String consumerTag,
                                           Envelope envelope,
                                           AMQP.BasicProperties properties,
                                           byte[] body) {

                    try {
                        long deliveryTag = envelope.getDeliveryTag();
                        // RabbitMQMessage message = new RabbitMQMessage(properties, body);
                        // get the sensor id from the properties
                        Map<String, Object> props = new HashMap<String, Object>();
                        if (properties != null && properties.getHeaders() != null) {
                            for (Map.Entry<String, Object> e : properties.getHeaders().entrySet()) {
                                props.put(e.getKey(), e.getValue().toString());
                            }
                        }

                        Message message = new Message(body, props);
                        channel.basicAck(deliveryTag, false);
                        handler.onMessage(message);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            });
            // save the name for deleting the queue
            queueDetailsMap.put(id, new QueueDetails(queueName, routingKey));
            return id;
        } catch (Exception e) {
            String msg = "could not open channel for exchange " + exchangeName;
            LOG.error(msg);
            throw new Exception(msg, e);
        }
    }

    public void stopListen(final String id) throws Exception {
        QueueDetails details = queueDetailsMap.get(id);
        if (details != null) {
            try {
                channel.queueUnbind(details.getQueueName(), exchangeName, details.getRoutingKey());
                channel.queueDelete(details.getQueueName(), true, true);
            } catch (IOException e) {
                String msg = "could not un-bind queue: " + details.getQueueName() + " for exchange " + exchangeName;
                LOG.error(msg);
                throw new Exception(msg, e);
            }
        }
    }

    private Connection createConnection() throws IOException {
        try {
            ConnectionFactory connectionFactory = new ConnectionFactory();
            connectionFactory.setUri(url);
            Connection connection = connectionFactory.newConnection();
            connection.addShutdownListener(new ShutdownListener() {
                public void shutdownCompleted(ShutdownSignalException cause) {
                }
            });
            LOG.info("connected to rabbitmq: " + connection + " for " + exchangeName);
            return connection;
        } catch (Exception e) {
            LOG.info("connection failed to rabbitmq: " + connection + " for " + exchangeName);
            return null;
        }
    }

    /**
     * Private class for holding some information about the consumers registered
     */
    private class QueueDetails {
        String queueName;

        String routingKey;

        private QueueDetails(String queueName, String routingKey) {
            this.queueName = queueName;
            this.routingKey = routingKey;
        }

        public String getQueueName() {
            return queueName;
        }

        public String getRoutingKey() {
            return routingKey;
        }
    }
}
