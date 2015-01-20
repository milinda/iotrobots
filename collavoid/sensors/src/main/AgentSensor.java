import cgl.iotcloud.core.*;
import cgl.iotcloud.core.msg.MessageContext;
import cgl.iotcloud.core.sensorsite.SiteContext;
import cgl.iotcloud.core.transport.Channel;
import cgl.iotcloud.core.transport.Direction;
import cgl.iotrobots.collavoid.commons.rmqmsg.IotRMQContext;
import cgl.iotrobots.collavoid.commons.rmqmsg.Methods_RMQ;
import cgl.iotrobots.collavoid.commons.rmqmsg.RMQContext;
import cgl.iotrobots.collavoid.commons.rmqmsg.Twist_;
import cgl.iotrobots.collavoid.commons.storm.Constant_storm;
import cgl.iotrobots.collavoid.controller.AgentControllerIot;
import org.ros.node.NodeConfiguration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingDeque;

public class AgentSensor extends AbstractSensor {
    public AgentControllerIot controllerIot;

    private static Logger LOG = LoggerFactory.getLogger(AgentSensor.class);

    public static void main(String[] args) {
        Map<String, String> properties = getProperties();
        String jarName = new File(AgentSensor.class.getProtectionDomain()
                .getCodeSource().getLocation().getPath()).getName();
        String noSensors = properties.get(Constant_storm.IotCloud.NO_SENSORS_ARG);
        String sites = properties.get(Constant_storm.IotCloud.SITES_ARG);

        int no = Integer.parseInt(noSensors);
        String []s = sites.split(" ");

        for (int i = 0; i < no; i++) {
            properties.put(Constant_storm.IotCloud.AGENT_INDEX,""+i);
            SensorSubmitter.submitSensor(properties, jarName,
                    AgentSensor.class.getCanonicalName(), Arrays.asList(s));
        }
    }

    private static Map<String,String> getProperties(){
        Map<String,String> res=new HashMap<>();
        res.put(Constant_storm.IotCloud.ROS_MASTER_URI, "http://localhost:11311");
        res.put(Constant_storm.IotCloud.LOCAL_IP_ARG, "localhost");
        res.put(Constant_storm.IotCloud.NO_SENSORS_ARG,"6");
        res.put(Constant_storm.IotCloud.SITES_ARG,"local");
        return res;
    }


    @Override
    public Configurator getConfigurator(Map map) {
        return new AgentSensorConfigurator();
    }

    @Override
    public void open(final SensorContext sensorContext) {
        controllerIot=new AgentControllerIot(sensorContext);
        NodeConfiguration nodeConfiguration = null;
        String localIp = (String) sensorContext.getProperty(Constant_storm.IotCloud.LOCAL_IP_ARG);
        String rosMaster = (String) sensorContext.getProperty(Constant_storm.IotCloud.ROS_MASTER_URI);
        try {
            nodeConfiguration = NodeConfiguration.newPublic(localIp, new URI(rosMaster));
        } catch (URISyntaxException e) {
            LOG.error("Failed to connect", e);
        }

        controllerIot.start(nodeConfiguration);

        startListen(sensorContext.getChannel(
                        Constant_storm.IotCloud.TRANSPORT,
                        Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT),
                new MessageReceiver() {
                    BlockingQueue<Twist_> velqueue = (BlockingQueue<Twist_>) sensorContext.getProperty(Constant_storm.IotCloud.VELOCITY_QUEUE);
                    @Override
                    public void onMessage(Object message) {
                        if (message instanceof MessageContext) {
                            byte[] body = ((MessageContext) message).getBody();
                            Twist_ vel = (Twist_) Methods_RMQ.deserialize(body, Twist_.class);

//                        String time = (String) ((MessageContext) message).getProperties().get("time");
//                        try {
//                            PrintWriter writer = new PrintWriter(new BufferedWriter(new FileWriter("/home/supun/dev/projects/LatencyTest.txt", true)));
//                            writer.println(System.currentTimeMillis() + " " + (System.currentTimeMillis() - Long.parseLong(time)));
//                            writer.close();
//                        } catch (FileNotFoundException e) {
//                            e.printStackTrace();
//                        }


                            velqueue.offer(vel);
//                        LOG.info("Message received " + message.toString());

                        } else {
                            LOG.error("Unexpected message");
                        }
                    }
                });
        LOG.info("Received request for opening sensor: {} with id: {}", sensorContext.getSensorID());
    }

    @Override
    public void close() {
        super.close();
    }

    private class AgentSensorConfigurator extends AbstractConfigurator {
        public Map<String, IotRMQContext> msgContexts;
        @Override
        public SensorContext configure(SiteContext siteContext, Map conf) {
            SensorContext context = new SensorContext(Constant_storm.IotCloud.SENSOR_NAME);
            String rosMaster = (String) conf.get(Constant_storm.IotCloud.ROS_MASTER_URI);
            String localIp = (String) conf.get(Constant_storm.IotCloud.LOCAL_IP_ARG);
            String index=(String) conf.get(Constant_storm.IotCloud.AGENT_INDEX);
            context.addProperty(Constant_storm.IotCloud.ROS_MASTER_URI, rosMaster);
            context.addProperty(Constant_storm.IotCloud.LOCAL_IP_ARG, localIp);
            context.addProperty(Constant_storm.IotCloud.AGENT_INDEX,index);
            BlockingQueue<Twist_> velqueue=new LinkedBlockingDeque<>();
            context.addProperty(Constant_storm.IotCloud.VELOCITY_QUEUE,velqueue);

            // self defined contexts
            msgContexts=new Constant_storm.IotMsgContexts(context.getSensorID()).Contexts;
            for (Map.Entry<String,IotRMQContext> e:msgContexts.entrySet()){
                Map Props = new HashMap();
                Props.put("exchange", e.getValue().EXCHANGE_NAME);
                Props.put("routingKey", e.getValue().ROUTING_KEY);
                Props.put("queueName", e.getValue().QUEUE_NAME);
                Channel channel;
                if (e.getKey().equals(Constant_storm.Components.VELOCITY_COMMAND_PUBLISHER_COMPONENT)||
                        e.getKey().equals(Constant_storm.Components.POSE_SHARE_PUB_COMPONENT)){
                    channel = createChannel(e.getKey(), Props, Direction.IN, 1024);
                }
                else{
                    channel = createChannel(e.getKey(), Props, Direction.OUT, 1024);
                }
                channel.setGrouped(true);
                context.addChannel(Constant_storm.IotCloud.TRANSPORT, channel);
            }

            return context;
        }
    }
}
