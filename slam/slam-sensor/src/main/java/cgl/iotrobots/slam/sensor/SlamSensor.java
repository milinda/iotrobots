package cgl.iotrobots.slam.sensor;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotrobots.utils.rabbitmq.RabbitMQReceiver;
import cgl.iotrobots.utils.rabbitmq.RabbitMQSender;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class SlamSensor extends AbstractSensor {
    public static final String LASE_SCAN_SENDER = "laseScanSender";
    public static final String MAP_RECEIVER = "mapReceiver";
    public static final String BEST_PARTICLE_RECEIVER = "bestParticleReceiver";
    public static final String SLAM_EXCHANGE = "slam_turtlebot";

    public static final String STORM_LASER_SCAN = "storm_laser_scan";
    public static final String STORM_MAP = "storm_map";
    public static final String STORM_BEST_PARTICLE = "storm_best_particle";

    public static final String URL_ARG = "url";
    public static final String SITES_ARG = "s";
    private static Logger LOG = LoggerFactory.getLogger(SlamSensor.class);

    public static final String BROKER_URL = "broker_url";

    private BlockingQueue frameReceivingQueue = new LinkedBlockingQueue();
    private BlockingQueue navDataReceivingQueue = new LinkedBlockingQueue();

    private BlockingQueue sendingQueue = new LinkedBlockingQueue();

    private int commandRecvCount = 0;

    private boolean run = true;

    private boolean simulator = true;

    private RabbitMQReceiver laserScanReceiver;
    private RabbitMQSender mapSender;
    private RabbitMQSender bestParticleSender;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        String sites = properties.get(SITES_ARG);
        String []s = sites.split(" ");

        String jarName = new File(SlamSensor.class.getProtectionDomain()
                .getCodeSource().getLocation().getPath()).getName();
        SensorSubmitter.submitSensor(properties, jarName,
                SlamSensor.class.getCanonicalName(), Arrays.asList(s));
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new STSensorConfigurator();
    }

    @Override
    public void open(SensorContext context) {
        final Channel sendChannel = context.getChannel("rabbitmq", LASE_SCAN_SENDER);
        final Channel bestChannel = context.getChannel("rabbitmq", BEST_PARTICLE_RECEIVER);
        final Channel mapChannel = context.getChannel("rabbitmq", MAP_RECEIVER);

        String brokerURL = (String) context.getProperty(BROKER_URL);

        if (simulator) {
            laserScanReceiver = new RabbitMQReceiver(frameReceivingQueue, "drone_frame", null, null, brokerURL);
            mapSender = new RabbitMQSender(navDataReceivingQueue, "drone_nav_data", null, null, brokerURL);
            bestParticleSender = new RabbitMQSender(sendingQueue, "drone", "control", "control", null, null, brokerURL);
        }

        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                while (run) {
                    try {
                        MessageContext messageContext = (MessageContext) frameReceivingQueue.take();
                        sendChannel.publish(messageContext);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t.start();


        Thread t2 = new Thread(new Runnable() {
            @Override
            public void run() {
                while (run) {
                    try {
                        MessageContext messageContext = (MessageContext) navDataReceivingQueue.take();
                        bestChannel.publish(messageContext);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        });
        t2.start();

        startListen(mapChannel, new MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    try {
                        commandRecvCount++;
                        System.out.println("Command received count: " + commandRecvCount);
                        sendingQueue.put(message);
                    } catch (InterruptedException e) {
                        LOG.error("Failed to put the message for sending", e);
                    }
                }
            }
        });

    }

    @Override
    public void close() {
        run = false;
        super.close();
        if (videoReceiver != null) {
            videoReceiver.stop();
        }
        if (navReceiver != null) {
            navReceiver.stop();
        }

        if (controlSender != null) {
            controlSender.stop();
        }
    }

    @SuppressWarnings("unchecked")
    private class STSensorConfigurator extends AbstractConfigurator {
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            String brokerUrl = (String) conf.get(BROKER_URL);

            SensorContext context = new SensorContext("drone_sensor");
            context.addProperty(BROKER_URL, brokerUrl);
            Map sendProps = new HashMap();
            sendProps.put("exchange", SLAM_EXCHANGE);
            sendProps.put("routingKey", STORM_DRONE_FRAME_ROUTING_KEY);
            sendProps.put("queueName", STORM_LASER_SCAN);
            Channel sendChannel = createChannel(LASE_SCAN_SENDER, sendProps, Direction.OUT, 1024);

            Map navSendProps = new HashMap();
            navSendProps.put("exchange", SLAM_EXCHANGE);
            navSendProps.put("routingKey", STORM_DRONE_NAV_DATA_ROUTING_KEY);
            navSendProps.put("queueName", STORM_MAP);
            Channel navSendChannel = createChannel(NAV_SENDER, navSendProps, Direction.OUT, 1024);

            Map receiveProps = new HashMap();
            receiveProps.put("queueName", STORM_BEST_PARTICLE);
            receiveProps.put("exchange", SLAM_EXCHANGE);
            receiveProps.put("routingKey", STORM_CONTROL_ROUTING_KEY);
            Channel receiveChannel = createChannel(MAP_RECEIVER, receiveProps, Direction.IN, 1024);

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);
            context.addChannel("rabbitmq", navSendChannel);

            return context;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption(URL_ARG, true, "URL of the AMQP Broker");
        options.addOption(SITES_ARG, true, "list of sites");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);

            String url = cmd.getOptionValue(URL_ARG);
            String sites = cmd.getOptionValue(SITES_ARG);

            conf.put(BROKER_URL, url);
            conf.put(SITES_ARG, sites);

            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
