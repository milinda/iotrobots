package cgl.iotrobots.collavoid.sensors;

import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotrobots.collavoid.commons.rmqmsg.IotRMQContext;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import com.esotericsoftware.kryo.Kryo;
import org.apache.commons.cli.*;
import org.ros.node.NodeConfiguration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;

public class CollAvoidSensor extends AbstractSensor {
    private static String SENSOR_NUMBER_ARG_NAME = "n";
    private static int nrobot=2;
    private static Logger LOG = LoggerFactory.getLogger(CollAvoidSensor.class);
    private static CollAvoidController controllerIot;

    public static void main(String[] args) {
        Map<String, String> properties = getProperties(args);
        String jarName = new File(CollAvoidSensor.class.getProtectionDomain()
                .getCodeSource().getLocation().getPath()).getName();
        String noSensors = properties.get(Constant_storm.IotCloud.SENSOR_NUMBER_KEY);
        String sites = properties.get(Constant_storm.IotCloud.SITES_KEY);

        if (noSensors!=null) {
            nrobot = Integer.parseInt(noSensors);
        }
        String[] s = sites.split(" ");

        for (int i = 0; i < nrobot; i++) {
            properties.put(Constant_storm.IotCloud.AGENT_INDEX, "" + i);
            SensorSubmitter.submitSensor(properties, jarName,
                    CollAvoidSensor.class.getCanonicalName(), Arrays.asList(s));
        }
    }

    private static Map<String, String> getProperties(String[] args) {
        Map<String, String> conf = new HashMap<String, String>();
        conf.put(Constant_storm.IotCloud.ROS_MASTER_URI_KEY, "http://localhost:11311");
        conf.put(Constant_storm.IotCloud.ROS_IP_KEY, "localhost");
        conf.put(Constant_storm.IotCloud.SITES_KEY, "local");

        Options options = new Options();
        options.addOption(SENSOR_NUMBER_ARG_NAME, true, "number of sensors");

        CommandLineParser commandLineParser = new BasicParser();
        try {
            CommandLine cmd = commandLineParser.parse(options, args);
            String n = cmd.getOptionValue(SENSOR_NUMBER_ARG_NAME);
            conf.put(Constant_storm.IotCloud.SENSOR_NUMBER_KEY, n);
            LOG.info("NO of sensors", conf.get(Constant_storm.IotCloud.SENSOR_NUMBER_KEY));
            return conf;
        } catch (ParseException e) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp("sensor", options);
        }
        return null;
    }

    @Override
    public Configurator getConfigurator(Map map) {
        return new AgentSensorConfigurator();
    }

    @Override
    public void open(final SensorContext sensorContext) {
        LOG.info("\n----------------Sensor for Simbad simulation opened!------------------\n");
        controllerIot = new CollAvoidController(sensorContext);
        NodeConfiguration nodeConfiguration = null;

        String localIp = (String) sensorContext.getProperty(Constant_storm.IotCloud.ROS_IP_KEY);
        String rosMaster = (String) sensorContext.getProperty(Constant_storm.IotCloud.ROS_MASTER_URI_KEY);
        try {
            nodeConfiguration = NodeConfiguration.newPublic(localIp, new URI(rosMaster));
        } catch (URISyntaxException e) {
            LOG.error("Failed to connect", e);
        }
        controllerIot.start(nodeConfiguration);

        startListen(sensorContext.getChannel(
                        Constant_storm.IotCloud.TRANSPORT,
                        Constant_storm.IotCloud.Channels.VELOCITY_PUBLISHER_CHANNEL),
                new MessageReceiver() {
                    BlockingQueue<Twist_> velqueue = (BlockingQueue<Twist_>) sensorContext.getProperty(Constant_storm.IotCloud.VELOCITY_QUEUE);
                    Kryo kryo = Methods_RMQ.getKryo();

                    @Override
                    public void onMessage(Object message) {
                        if (message instanceof MessageContext) {
                            byte[] body = ((MessageContext) message).getBody();
                            Twist_ vel = (Twist_) Methods_RMQ.deSerialize(kryo, body);
                            velqueue.offer(vel);
                        } else {
                            LOG.error("Unexpected message");
                        }
                    }
                });

        startListen(sensorContext.getChannel(
                        Constant_storm.IotCloud.TRANSPORT,
                        Constant_storm.IotCloud.Channels.POSE_SHARE_PUB_COMPONENT),
                new MessageReceiver() {
                    @Override
                    public void onMessage(Object message) {
                        // do nothing just ack the message
                    }
                });

        LOG.info("Received request for opening sensor: {} with id: {}", sensorContext.getSensorID());
    }

    @Override
    public void close() {
        super.close();
        controllerIot.close();
    }

    private class AgentSensorConfigurator extends AbstractConfigurator {
        public Map<String, IotRMQContext> rmqContexts;

        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            SensorContext context = new SensorContext(Constant_storm.IotCloud.SENSOR_NAME);
            String rosMaster = (String) conf.get(Constant_storm.IotCloud.ROS_MASTER_URI_KEY);
            String localIp = (String) conf.get(Constant_storm.IotCloud.ROS_IP_KEY);
            String index = (String) conf.get(Constant_storm.IotCloud.AGENT_INDEX);
            context.addProperty(Constant_storm.IotCloud.ROS_MASTER_URI_KEY, rosMaster);
            context.addProperty(Constant_storm.IotCloud.ROS_IP_KEY, localIp);
            context.addProperty(Constant_storm.IotCloud.AGENT_INDEX, index);
            BlockingQueue<Twist_> velqueue = new LinkedBlockingDeque<Twist_>();
            context.addProperty(Constant_storm.IotCloud.VELOCITY_QUEUE, velqueue);

            // contexts for channels
            rmqContexts = new Constant_storm.IotMsgContexts().Contexts;
            for (Map.Entry<String, IotRMQContext> e : rmqContexts.entrySet()) {
                Map Props = new HashMap();
                Props.put("exchange", e.getValue().EXCHANGE_NAME);
                Props.put("routingKey", e.getValue().ROUTING_KEY);
                Props.put("queueName", e.getValue().QUEUE_NAME);
                Channel channel;
                if (e.getKey().equals(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT)) {
                    channel = createChannel(e.getValue().CHANNEL, Props, Direction.IN, 1024);
                    channel.setGrouped(false);
                } else if (e.getKey().equals(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT)) {
                    channel = createChannel(e.getValue().CHANNEL, Props, Direction.IN, 1024);
                    channel.setGrouped(true);
                } else {
                    channel = createChannel(e.getValue().CHANNEL, Props, Direction.OUT, 1024);
                    channel.setGrouped(true);
                }

                context.addChannel(Constant_storm.IotCloud.TRANSPORT, channel);
            }

            return context;
        }
    }
}

