package cgl.iotrobots.collavoid.commons.rabbitmq;

import com.rabbitmq.client.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class RabbitMQSender {
    public static final int DEFAULT_PRE_FETCH = 64;

    private static Logger LOG = LoggerFactory.getLogger(RabbitMQSender.class);

    private Connection connection;

    private Channel channel;

    private QueueingConsumer consumer;

    private String consumerTag;

    private String exchangeName;

    private int prefetchCount = DEFAULT_PRE_FETCH;

    private boolean isReQueueOnFail = false;

    private String url;

    public RabbitMQSender(String url, String exchangeName) {
        this.exchangeName = exchangeName;
        this.url = url;
    }

    public void setPrefetchCount(int prefetchCount) {
        this.prefetchCount = prefetchCount;
    }

    public void setReQueueOnFail(boolean isReQueueOnFail) {
        this.isReQueueOnFail = isReQueueOnFail;
    }

    private void reset() {
        consumerTag = null;
    }

    private void reInitIfNecessary() throws Exception {
        if (consumerTag == null || consumer == null) {
            close();
            open("fanout");
        }
    }

    public void close() {
        LOG.info("Closing channel to exchange {}", exchangeName);
        try {
            if (channel != null && channel.isOpen()) {
                if (consumerTag != null) {
                    channel.basicCancel(consumerTag);
                }
                channel.close();
            }
        } catch (Exception e) {
            LOG.debug("error closing channel and/or cancelling consumer", e);
        }
        try {
            LOG.info("closing connection to rabbitmq: " + connection);
            connection.close();
        } catch (Exception e) {
            LOG.debug("error closing connection", e);
        }
        consumer = null;
        consumerTag = null;
        channel = null;
        connection = null;
    }

    public void open(String exchangeType) throws Exception {
        try {
            connection = createConnection();
            channel = connection.createChannel();
            if (prefetchCount > 0) {
                LOG.info("setting basic.qos / prefetch count to " + prefetchCount + " for " + exchangeName);
                channel.basicQos(prefetchCount);
            }
            channel.exchangeDeclare(exchangeName, exchangeType, true);//set to true for test

        } catch (Exception e) {
            reset();
            String msg = "could not open channel for exchange " + exchangeName;
            LOG.error(msg);
            throw new Exception(msg, e);
        }
    }

    public void send(Message input, String routingKey) throws Exception {
        try {
            Map<String, Object> props = new HashMap<String, Object>();
            for (Map.Entry<String, Object> e : input.getProperties().entrySet()) {
                props.put(e.getKey(), e.getValue());
            }
            channel.basicPublish(exchangeName, routingKey,
                    new AMQP.BasicProperties.Builder().headers(props).build(), input.getBody());
        } catch (IOException e) {
            String msg = "Failed to publish message to exchange: " + exchangeName;
            LOG.error(msg, e);
            throw new Exception(msg, e);
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

    public void ackMessage(Long msgId) throws Exception {
        try {
            channel.basicAck(msgId, false);
        } catch (ShutdownSignalException sse) {
            reset();
            String msg = "shutdown signal received while attempting to ack message";
            LOG.error(msg, sse);
            throw new Exception(msg, sse);
        } catch (Exception e) {
            String s = "could not ack for msgId: " + msgId;
            LOG.error(s, e);
            throw new Exception(s, e);
        }
    }

    public void failMessage(Long msgId) throws Exception {
        if (isReQueueOnFail) {
            failWithRedelivery(msgId);
        } else {
            deadLetter(msgId);
        }
    }

    public void failWithRedelivery(Long msgId) throws Exception {
        try {
            channel.basicReject(msgId, true);
        } catch (ShutdownSignalException sse) {
            reset();
            String msg = "shutdown signal received while attempting to fail with redelivery";
            LOG.error(msg, sse);
            throw new Exception(msg, sse);
        } catch (Exception e) {
            String msg = "could not fail with redelivery for msgId: " + msgId;
            LOG.error(msg, e);
            throw new Exception(msg, e);
        }
    }

    public void deadLetter(Long msgId) throws Exception {
        try {
            channel.basicReject(msgId, false);
        } catch (ShutdownSignalException sse) {
            reset();
            String msg = "shutdown signal received while attempting to fail with no redelivery";
            LOG.error(msg, sse);
            throw new Exception(msg, sse);
        } catch (Exception e) {
            String msg = "could not fail with dead-lettering (when configured) for msgId: " + msgId;
            LOG.error(msg, e);
            throw new Exception(msg, e);
        }
    }
}
