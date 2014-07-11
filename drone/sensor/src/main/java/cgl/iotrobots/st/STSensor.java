package cgl.iotrobots.st;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class STSensor extends AbstractSensor {
    private static Logger LOG = LoggerFactory.getLogger(STSensor.class);

    public static final String BROKER_URL = "broker_url";

    private DroneMessageReceiver videoReceiver;

    private DroneMessageSender controlSender;

    BlockingQueue receivingQueue = new LinkedBlockingQueue();
    BlockingQueue sendingQueue = new LinkedBlockingQueue();

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        SensorSubmitter.submitSensor(properties, "drone-sensor-1.0-SNAPSHOT-jar-with-dependencies.jar", STSensor.class.getCanonicalName(), Arrays.asList("local-1"));
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        final Channel sendChannel = context.getChannel("rabbitmq", "sender");
        final Channel receiveChannel = context.getChannel("rabbitmq", "receiver");

        String brokerURL = (String) context.getProperty(BROKER_URL);

        videoReceiver = new DroneMessageReceiver(receivingQueue, "drone_frame", null, null, brokerURL);
        videoReceiver.setExchangeName("drone");
        videoReceiver.setRoutingKey("drone_frame");
        videoReceiver.start();


        controlSender = new DroneMessageSender(sendingQueue, "drone", "control", "control", null, null, brokerURL);
        controlSender.start();

        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    try {
                        MessageContext messageContext = (MessageContext) receivingQueue.take();
                        sendChannel.publish(messageContext);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t.start();

        startSend(sendChannel, receivingQueue);

        startListen(receiveChannel, new MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    try {
                        sendingQueue.put(message);
                    } catch (InterruptedException e) {
                        LOG.error("Failed to put the message for sending", e);
                    }
                }
            }
        });
        LOG.info("Received request {}", context.getId());
    }

    @Override
    public void close() {
        super.close();
        if (videoReceiver != null) {
            videoReceiver.stop();
        }
    }

    @SuppressWarnings("unchecked")
    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String brokerUrl = (String) conf.get(BROKER_URL);

            SensorContext context = new SensorContext(new SensorId("turtle_sensor", "general"));
            context.addProperty(BROKER_URL, brokerUrl);
            Map sendProps = new HashMap();
            sendProps.put("exchange", "storm_drone");
            sendProps.put("routingKey", "storm_drone_frame");
            sendProps.put("queueName", "storm_drone_frame");
            Channel sendChannel = createChannel("sender", sendProps, Direction.OUT, 1024);

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", "storm_control");
            receiveProps.put("exchange", "storm_drone");
            receiveProps.put("routingKey", "storm_control");
            Channel receiveChannel = createChannel("receiver", receiveProps, Direction.IN, 1024);

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);

            return context;
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
