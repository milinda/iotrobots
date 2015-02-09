package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.utils.rabbitmq.Message;
import cgl.iotrobots.utils.rabbitmq.RabbitMQSender;
import org.ros.node.AbstractNodeMain;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class SimUtils {
    private static Random r = new Random();

    public static void sendControl(RabbitMQSender sender) {
        byte[] body = "start".getBytes();
        Map<String, Object> props = new HashMap<String, Object>();
        props.put("time", System.currentTimeMillis());
        props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());
        Message message = new Message(body, props);
        try {
            sender.send(message, "test.test.control");
            Thread.sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static double gausianNoise(double variance, double mean) {
        double noise = r.nextGaussian() * Math.sqrt(variance) + mean;
        return noise;
    }

    static void connectToRos(AbstractNodeMain node) {
        // register with ros_java
        NodeConfiguration nodeConfiguration;
        try {
//            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.93.220:11311"));
//            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.95.50:11311"));
            nodeConfiguration = NodeConfiguration.newPublic("192.168.1.6", new URI("http://192.168.1.6:11311"));
            NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
            nodeMainExecutor.execute(node, nodeConfiguration);
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
    }
}
