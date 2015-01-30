package cgl.iotrobots.slam.sensor;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotrobots.utils.rabbitmq.*;
import org.apache.commons.cli.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class SlamSensor extends AbstractSensor {
    public static final String LASE_SCAN_SENDER = "laseScanSender";
    public static final String CONTROL_SENDER = "controlSender";
    public static final String MAP_RECEIVER = "mapReceiver";
    public static final String BEST_PARTICLE_RECEIVER = "bestParticleReceiver";
    public static final String SLAM_EXCHANGE = "slam_sensor";

    public static final String STORM_LASER_SCAN = "storm_laser_scan";
    public static final String STORM_MAP = "storm_map";
    public static final String STORM_BEST_PARTICLE = "storm_best_particle";

    public static final String SIMULATOR_EXCHANGE = "simulator";

    public static final String URL_ARG = "url";
    public static final String SITES_ARG = "s";
    public static final String SIMULATOR_ARG = "sim";

    private static Logger LOG = LoggerFactory.getLogger(SlamSensor.class);

    public static final String BROKER_URL = "broker_url";

    private int commandRecvCount = 0;
    private boolean run = true;

    private boolean simulator = true;

    private RabbitMQReceiver laserScanReceiver;
    private RabbitMQReceiver controlReceiver;
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
        final Channel controlChannel = context.getChannel("rabbitmq", CONTROL_SENDER);
        final Channel bestChannel = context.getChannel("rabbitmq", BEST_PARTICLE_RECEIVER);
        final Channel mapChannel = context.getChannel("rabbitmq", MAP_RECEIVER);

        String brokerURL = (String) context.getProperty(BROKER_URL);
        simulator = Boolean.parseBoolean(context.getProperty(SIMULATOR_ARG).toString());

        if (simulator) {
            try {
                laserScanReceiver = new RabbitMQReceiver(brokerURL, "simbard_laser");
                controlReceiver = new RabbitMQReceiver(brokerURL, "simbard_control");
                mapSender = new RabbitMQSender(brokerURL, "simbard_map");
                bestParticleSender = new RabbitMQSender(brokerURL, "simbard_best");
                mapSender.open();
                bestParticleSender.open();

                laserScanReceiver.listen(new LaserScanReceiver(sendChannel));
                controlReceiver.listen(new ControlReceiver(controlChannel));
            } catch (Exception e) {
                LOG.error("Failed to create rabbitmq receiver", e);
            }
        }

        startListen(mapChannel, new MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    Message m = new Message(((MessageContext) message).getBody(), ((MessageContext) message).getProperties());
                    commandRecvCount++;
                    System.out.println("Map received count: " + commandRecvCount);
                    try {
                        mapSender.send(m, "test.test.map");
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });

        startListen(bestChannel, new MessageReceiver() {
            @Override
            public void onMessage(Object message) {
                if (message instanceof MessageContext) {
                    Message m = new Message(((MessageContext) message).getBody(), ((MessageContext) message).getProperties());
                    commandRecvCount++;
                    System.out.println("Best received count: " + commandRecvCount);
                    try {
                        bestParticleSender.send(m, "test.test.best");
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }
        });
    }

    private class LaserScanReceiver implements MessageHandler {
        private Channel sendChannel;

        private LaserScanReceiver(Channel sendChannel) {
            this.sendChannel = sendChannel;
        }

        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.laser_scan");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.laser_scan");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            MessageContext messageContext = new MessageContext("default", message.getBody(), message.getProperties());
            this.sendChannel.publish(messageContext);
        }
    }

    private class ControlReceiver implements MessageHandler {
        private Channel sendChannel;

        private ControlReceiver(Channel sendChannel) {
            this.sendChannel = sendChannel;
        }

        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.control");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.control");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            MessageContext messageContext = new MessageContext("default", message.getBody(), message.getProperties());
            this.sendChannel.publish(messageContext);
        }
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

            SensorContext context = new SensorContext("data_sensor");
            context.addProperty(BROKER_URL, brokerUrl);
            context.addProperty(SIMULATOR_ARG, conf.get(SIMULATOR_ARG));
            Map sendProps = new HashMap();
            sendProps.put("exchange", SLAM_EXCHANGE);
            sendProps.put("routingKey", STORM_LASER_SCAN);
            sendProps.put("queueName", STORM_LASER_SCAN);
            Channel sendChannel = createChannel(LASE_SCAN_SENDER, sendProps, Direction.OUT, 1024);

            Map controlProps = new HashMap();
            controlProps.put("exchange", SLAM_EXCHANGE);
            controlProps.put("routingKey", STORM_LASER_SCAN);
            controlProps.put("queueName", STORM_LASER_SCAN);
            Channel controlChannel = createChannel(CONTROL_SENDER, controlProps, Direction.OUT, 1024);

            Map mapReceiveProps = new HashMap();
            mapReceiveProps.put("exchange", SLAM_EXCHANGE);
            mapReceiveProps.put("routingKey", STORM_MAP);
            mapReceiveProps.put("queueName", STORM_MAP);
            Channel navSendChannel = createChannel(MAP_RECEIVER, mapReceiveProps, Direction.OUT, 1024);

            Map bestReceiveProps = new HashMap();
            bestReceiveProps.put("queueName", STORM_BEST_PARTICLE);
            bestReceiveProps.put("exchange", SLAM_EXCHANGE);
            bestReceiveProps.put("routingKey", STORM_BEST_PARTICLE);
            Channel receiveChannel = createChannel(BEST_PARTICLE_RECEIVER, bestReceiveProps, Direction.IN, 1024);

            context.addChannel("rabbitmq", sendChannel);
            context.addChannel("rabbitmq", receiveChannel);
            context.addChannel("rabbitmq", navSendChannel);
            context.addChannel("rabbitmq", controlChannel);

            return context;
        }
    }

    private static Map<String, String> getProperties(String []args) {
        Map<String, String> conf = new HashMap<String, String>();
        Options options = new Options();
        options.addOption(URL_ARG, true, "URL of the AMQP Broker");
        options.addOption(SITES_ARG, true, "list of sites");
        options.addOption(SIMULATOR_ARG, false, "Weather to use simulator");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);

            String url = cmd.getOptionValue(URL_ARG);
            String sites = cmd.getOptionValue(SITES_ARG);
            boolean sim = cmd.hasOption(SIMULATOR_ARG);

            conf.put(BROKER_URL, url);
            conf.put(SITES_ARG, sites);
            conf.put(SIMULATOR_ARG, Boolean.toString(sim));

            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options );
        }
        return null;
    }
}
