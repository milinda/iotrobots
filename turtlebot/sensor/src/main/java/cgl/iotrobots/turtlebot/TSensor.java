package cgl.iotrobots.turtlebot;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotcloud.core.transport.IdentityConverter;
import cgl.iotcloud.core.transport.MessageConverter;
import cgl.iotcloud.transport.rabbitmq.RabbitMQMessage;
import cgl.iotrobots.turtlebot.commons.CommonsUtils;
import cgl.iotrobots.turtlebot.commons.KinectMessageReceiver;
import cgl.iotrobots.turtlebot.commons.Motion;
import com.google.common.collect.Lists;
import org.apache.commons.cli.*;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.NodeConfiguration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class TSensor extends AbstractSensor {
    private static Logger LOG = LoggerFactory.getLogger(TSensor.class);

    public static final String BROKER_URL = "broker_url";

    private TurtleController controller;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        SensorSubmitter.submitSensor(properties, "iotrobots-turtlebot-1.0-SNAPSHOT-jar-with-dependencies.jar", TSensor.class.getCanonicalName(), Arrays.asList("local-1"));
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        BlockingQueue receivingQueue = new LinkedBlockingQueue();
        final Channel sendChannel = context.getChannel("rabbitmq", "sender");
        final Channel receiveChannel = context.getChannel("rabbitmq", "receiver");

        // register with ros_java
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(""));
        NodeConfiguration nodeConfiguration = null;
        try {
            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.102", new URI("http://149.160.205.153:11311"));
        } catch (URISyntaxException e) {
            LOG.error("Failed to connect", e);
        }
        controller = new TurtleController();
        controller.start(nodeConfiguration);

        String brokerURL = (String) context.getProperty(BROKER_URL);
        KinectMessageReceiver receiver = new KinectMessageReceiver(receivingQueue, "kinect_controller", null, null, brokerURL);
        receiver.setExchangeName("kinect_frames");
        receiver.setRoutingKey("kinect_controller");
        receiver.start();

        startSend(sendChannel, receivingQueue);

        startListen(receiveChannel, new cgl.iotcloud.core.MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof Motion) {
                    controller.setMotion((Motion) message);
                    System.out.println("Message received " + message.toString());
                } else {
                    System.out.println("Unexpected message");
                }
            }
        });
        LOG.info("Received request {}", context.getId());
    }

    @Override
    public void close() {
        super.close();
        controller.shutDown();
    }

    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String brokerUrl = (String) conf.get(BROKER_URL);

            SensorContext context = new SensorContext(new SensorId("turtle_sensor", "general"));
            context.addProperty(BROKER_URL, brokerUrl);
            Map sendProps = new HashMap();
            sendProps.put("exchange", "turtle_sensor");
            sendProps.put("routingKey", "kinect");
            sendProps.put("queueName", "kinect");
            Channel sendChannel = createChannel("sender", sendProps, Direction.OUT, 1024, new IdentityConverter());

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", "control");
            receiveProps.put("exchange", "turtle_sensor");
            receiveProps.put("routingKey", "control");
            Channel receiveChannel = createChannel("receiver", receiveProps, Direction.IN, 1024, new ControlConverter());

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);

            return context;
        }
    }

    private class ControlConverter implements MessageConverter {
        @Override
        public Object convert(Object input, Object context) {
            if (input instanceof RabbitMQMessage) {
                try {
                    return CommonsUtils.jsonToMotion(((RabbitMQMessage) input).getBody());
                } catch (IOException e) {
                    LOG.error("Failed to convert the message to a Motion", e);
                    return null;
                }
            }
            return null;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption("url", true, "URL of the AMQP Broker");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);

            String url = cmd.getOptionValue("url");
            conf.put(BROKER_URL, url);

            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
