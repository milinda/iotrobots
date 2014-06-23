package cgl.iotrobots.turtlebot;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotcloud.core.transport.IdentityConverter;
import cgl.iotcloud.core.transport.MessageConverter;
import cgl.iotcloud.transport.rabbitmq.RabbitMQMessage;
import cgl.iotrobots.turtlebot.commons.KinectMessageReceiver;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class TSensor extends AbstractSensor {
    private static Logger LOG = LoggerFactory.getLogger(TSensor.class);

    public static final String SEND_QUEUE = "send_queue";
    public static final String RECEIVE_QUEUE = "recv_queue";

    public static final String SEND_EXCHANGE = "send_exchange";
    public static final String SEND_ROUTING_KEY = "send_routing_key";

    public static final String RECV_EXCHANGE = "recv_exchange";
    public static final String RECV_ROUTING_KEY = "recv_routing_key";

    public static final String SENSOR_NAME = "name";
    public static final String BROKER_URL = "broker_url";

    private SensorContext context;

    private BlockingQueue receivingQueue;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        SensorSubmitter.submitSensor(properties, "", TSensor.class.getCanonicalName(), Arrays.asList("local"));
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

        String brokerURL = (String) context.getProperty(BROKER_URL);

        KinectMessageReceiver receiver = new KinectMessageReceiver(receivingQueue, "kinect_controller", null, null, "amqp://localhost:5672");
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


    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String exchange = (String) conf.get(SEND_EXCHANGE);
            String sendQueue = (String) conf.get(SEND_QUEUE);
            String routingKey = (String) conf.get(SEND_ROUTING_KEY);

            String recvExchange = (String) conf.get(RECV_EXCHANGE);
            String recvQueue = (String) conf.get(RECEIVE_QUEUE);
            String recvRoutingKey = (String) conf.get(RECV_ROUTING_KEY);

            String brokerUrl = (String) conf.get(BROKER_URL);
            String sensorName = (String) conf.get(SENSOR_NAME);

            SensorContext context = new SensorContext(new SensorId(sensorName, "general"));
            context.addProperty(BROKER_URL, brokerUrl);
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
                return null;
            }
            return null;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption("url", true, "URL of the AMQP Broker");
        options.addOption("qr", true, "Receive Queue name");
        options.addOption("qs", true, "Send Queue name");
        options.addOption("ex", true, "Exchange");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);

            String url = cmd.getOptionValue("url");
            conf.put(BROKER_URL, url);

            String recvQueueName = cmd.getOptionValue("qr");
            conf.put(RECEIVE_QUEUE, recvQueueName);
            conf.put(RECV_ROUTING_KEY, recvQueueName);

            String sendQueueName = cmd.getOptionValue("qs");
            conf.put(SEND_QUEUE, sendQueueName);
            conf.put(SEND_ROUTING_KEY, sendQueueName);

            String exchange = cmd.getOptionValue("ex");
            conf.put(SEND_EXCHANGE, exchange);
            conf.put(RECV_EXCHANGE, exchange);

            conf.put(SENSOR_NAME, "TurtleSensor");
            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
