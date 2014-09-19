import com.rabbitmq.client.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class TurtleMessageReceiver {
    private Channel channel;

    private Connection conn;

    private String queueName;

    private boolean autoAck = false;

    private Address[]addresses;

    private String url;

    private ExecutorService executorService;

    private String exchangeName;

    private String routingKey;

    public TurtleMessageReceiver(String queueName,
                                 ExecutorService executorService,
                                 Address []addresses,
                                 String url) {
        this.executorService = executorService;
        this.queueName = queueName;
        this.addresses = addresses;
        this.url = url;
    }

    public void start() {
        try {
            ConnectionFactory factory = new ConnectionFactory();
            if (addresses == null) {
                factory.setUri(url);
                if (executorService != null) {
                    conn = factory.newConnection(executorService);
                } else {
                    conn = factory.newConnection();
                }
            } else {
                if (executorService != null) {
                    conn = factory.newConnection(executorService, addresses);
                } else {
                    conn = factory.newConnection(addresses);
                }
            }

            channel = conn.createChannel();

            if (exchangeName != null && routingKey != null) {
                channel.exchangeDeclare(exchangeName, "fanout", true);
                channel.queueDeclare(this.queueName, true, false, false, null);
                channel.queueBind(queueName, exchangeName, routingKey);
            }

            channel.basicConsume(queueName, autoAck, "myConsumerTag",
                    new DefaultConsumer(channel) {
                        @Override
                        public void handleDelivery(String consumerTag,
                                                   Envelope envelope,
                                                   AMQP.BasicProperties properties,
                                                   byte[] body)
                                throws IOException {
                            long deliveryTag = envelope.getDeliveryTag();
                            Map<String, Object> props = new HashMap<String, Object>();
                            if (properties != null && properties.getHeaders() != null) {
                                for (Map.Entry<String, Object> e : properties.getHeaders().entrySet()) {
                                    props.put(e.getKey(), e.getValue());
                                }
                            }
                            String time = (String) props.get("time");
                            System.out.println(System.currentTimeMillis() - Long.parseLong(time));
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

    public void stop() {
        try {
            if (channel != null) {
                channel.close();
            }
            if (conn != null) {
                conn.close();
            }
        } catch (IOException e) {
            System.out.println("Error closing the rabbit MQ connection" + e);
        }
    }

    public void setRoutingKey(String routingKey) {
        this.routingKey = routingKey;
    }

    public void setExchangeName(String exchangeName) {
        this.exchangeName = exchangeName;
    }
}
