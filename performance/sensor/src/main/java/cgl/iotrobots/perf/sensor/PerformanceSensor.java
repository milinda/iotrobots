package cgl.iotrobots.perf.sensor;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class PerformanceSensor extends AbstractSensor {
    public static final String TURTLE_STORM_FRAMES_ROUTING_KEY = "turtle_storm_frames";
    public static final String TURTLE_STORM_FRAMES_QUEUE_NAME = "turtle_storm_frames";
    public static final String TURTLE_STORM_CONTROL_QUEUE_NAME = "turtle_storm_control";
    public static final String TURTLE_STORM_CONTROL_ROUTING_KEY = "turtle_storm_control";
    public static final String TURTLE_EXCHANGE = "turtle";
    public static final String CMD_RECV = "cmd_recv";
    public static final String FRAME_SENDER = "frame_sender";

    private static Logger LOG = LoggerFactory.getLogger(PerformanceSensor.class);

    public static final String BROKER_URL = "broker_url";

    public static final String MODE_ARG = "mode";
    public static final String URL_ARG = "url";
    public static final String LOCAL_IP_ARG = "local_ip";
    public static final String ROS_MASTER_ARG = "ros_master";

    private boolean run = true;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        SensorSubmitter.submitSensor(properties, new java.io.File(PerformanceSensor.class.getProtectionDomain()
                .getCodeSource().getLocation().getPath()).getName(),
                PerformanceSensor.class.getCanonicalName(), Arrays.asList("local"));
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        final BlockingQueue receivingQueue = new LinkedBlockingQueue();
        final Channel sendChannel = context.getChannel("rabbitmq", FRAME_SENDER);
        final Channel receiveChannel = context.getChannel("rabbitmq", CMD_RECV);

        // startSend(sendChannel, receivingQueue);
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                while (run) {
                    try {
                        byte[] body = (byte[]) receivingQueue.take();
                        Map<String, Object> props = new HashMap<String, Object>();
                        props.put("time", Long.toString(System.currentTimeMillis()));

                        sendChannel.publish(body, props);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t.start();

        startListen(receiveChannel, new cgl.iotcloud.core.MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    LOG.info("Message received " + message.toString());

                } else {
                    LOG.error("Unexpected message");
                }
            }
        });
        LOG.info("Received request for opening sensor: {} with id: {}", context.getSensorID());
    }

    @Override
    public void close() {
        run = false;
        super.close();
    }

    @SuppressWarnings("unchecked")
    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String brokerUrl = (String) conf.get(BROKER_URL);
            String rosMaster = (String) conf.get(ROS_MASTER_ARG);
            String localIp = (String) conf.get(LOCAL_IP_ARG);
            String mode = (String) conf.get(MODE_ARG);

            SensorContext context = new SensorContext("turtle_sensor");
            context.addProperty(BROKER_URL, brokerUrl);
            context.addProperty(ROS_MASTER_ARG, rosMaster);
            context.addProperty(LOCAL_IP_ARG, localIp);
            context.addProperty(MODE_ARG, mode);

            Map sendProps = new HashMap();
            sendProps.put("exchange", TURTLE_EXCHANGE);
            sendProps.put("routingKey", TURTLE_STORM_FRAMES_ROUTING_KEY);
            sendProps.put("queueName", TURTLE_STORM_FRAMES_QUEUE_NAME);
            Channel sendChannel = createChannel(FRAME_SENDER, sendProps, Direction.OUT, 1024);

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", TURTLE_STORM_CONTROL_QUEUE_NAME);
            receiveProps.put("exchange", TURTLE_EXCHANGE);
            receiveProps.put("routingKey", TURTLE_STORM_CONTROL_ROUTING_KEY);
            Channel receiveChannel = createChannel(CMD_RECV, receiveProps, Direction.IN, 1024);

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);

            return context;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption(URL_ARG, true, "URL of the AMQP Broker");
        options.addOption(ROS_MASTER_ARG, true, "Ros master URL");
        options.addOption(LOCAL_IP_ARG, true, "Local IP address");
        options.addOption(MODE_ARG, true, "possible options are (nt, t) nt means without connecting to turtle");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);

            String url = cmd.getOptionValue(URL_ARG);
            String rosMaster = cmd.getOptionValue(ROS_MASTER_ARG);
            String localIp = cmd.getOptionValue(LOCAL_IP_ARG);
            String mode = cmd.getOptionValue(MODE_ARG);

            System.out.println(url);
            System.out.println(rosMaster);
            System.out.println(localIp);

            conf.put(BROKER_URL, url);
            conf.put(ROS_MASTER_ARG, rosMaster);
            conf.put(LOCAL_IP_ARG, localIp);
            conf.put(MODE_ARG, mode);

            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
