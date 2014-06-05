package cgl.iotrobots.st;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotcloud.core.transport.IdentityConverter;
import cgl.iotcloud.core.transport.MessageConverter;
import cgl.iotcloud.transport.rabbitmq.RabbitMQMessage;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class STSensor extends AbstractSensor {
    private static Logger LOG = LoggerFactory.getLogger(STSensor.class);

    public static final String SEND_QUEUE_NAME_PROP = "send_queue";
    public static final String RECEIVE_QUEUE_PROP = "recv_queue";

    public static final String SEND_INTERVAL = "send_interval";
    public static final String FILE_NAME = "file_name";

    public static final String SEND_EXCHANGE_NAME_PROP = "send_exchange";
    public static final String SEND_ROUTING_KEY_PROP = "send_routing_key";

    public static final String RECV_EXCHANGE_NAME_PROP = "recv_exchange";
    public static final String RECV_ROUTING_KEY_PROP = "recv_routing_key";

    public static final String SERVER = "server";
    public static final String SENSOR_NAME = "name";

    private SensorContext context;

    private BlockingQueue receivingQueue;

    public static void main(String[] args) {

    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        this.context = context;

        receivingQueue = new LinkedBlockingQueue();

        final Channel sendChannel = context.getChannel("rabbitmq", "sender");
        final Channel receiveChannel = context.getChannel("rabbitmq", "receiver");

        MessageReceiver receiver = new MessageReceiver(receivingQueue, "task_queue", null, null, "amqp://149.160.213.3:5672");
        receiver.start();

        startSend(sendChannel, receivingQueue);

        startListen(receiveChannel, new cgl.iotcloud.core.MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof RabbitMQMessage) {
                    LOG.info("Message received " + Arrays.toString(((RabbitMQMessage) message).getBody()));
                } else {
                    System.out.println("Unexpected message");
                }
            }
        });
        LOG.info("Received request {}", this.context.getId());
    }

    @Override
    public void close() {

    }

    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            SensorContext context = new SensorContext(new SensorId("rabbitChat", "general"));
            String exchange = (String) conf.get(SEND_EXCHANGE_NAME_PROP);
            String sendQueue = (String) conf.get(SEND_QUEUE_NAME_PROP);

            String recvExchange = (String) conf.get(RECV_EXCHANGE_NAME_PROP);

            String recvQueue = (String) conf.get(RECEIVE_QUEUE_PROP);
            String routingKey = (String) conf.get(SEND_ROUTING_KEY_PROP);
            String recvRoutingKey = (String) conf.get(RECV_ROUTING_KEY_PROP);
            String fileName = (String) conf.get(FILE_NAME);

            String sendInterval = (String) conf.get(SEND_INTERVAL);
            int interval = Integer.parseInt(sendInterval);
            context.addProperty(SEND_INTERVAL, interval);
            context.addProperty(FILE_NAME, fileName);

            Map sendProps = new HashMap();
            sendProps.put("exchange", exchange);
            sendProps.put("routingKey", routingKey);
            sendProps.put("queueName", sendQueue);
            Channel sendChannel = createChannel("sender", sendProps, Direction.OUT, 1024, new FileContentToRabbitMessageConverter());

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", recvQueue);
            receiveProps.put("exchange", recvExchange);
            receiveProps.put("routingKey", recvRoutingKey);
            Channel receiveChannel = createChannel("receiver", receiveProps, Direction.IN, 1024, new IdentityConverter());

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);

            return context;
        }
    }

    private class FileContentMessage {
        private String content;

        private FileContentMessage(String content) {
            this.content = content;
        }

        public String getContent() {
            return content;
        }
    }

    private class FileContentToRabbitMessageConverter implements MessageConverter {
        @Override
        public Object convert(Object input, Object context) {
            if (input instanceof FileContentMessage) {
                long currentTime = System.currentTimeMillis();
                String send = currentTime + "\r\n" + ((FileContentMessage) input).getContent();
                RabbitMQMessage message = new RabbitMQMessage(null, send.getBytes());
                return message;
            }
            return null;
        }
    }
}
