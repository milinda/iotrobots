package cgl.iotrobots.slam.utils;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.utils.rabbitmq.Message;
import cgl.iotrobots.utils.rabbitmq.RabbitMQSender;
import geometry_msgs.Quaternion;
import org.ros.node.AbstractNodeMain;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.*;

public class TurtleUtils {
    private static Random r = new Random();

    public static LaserScan turtleScanToLaserScan(sensor_msgs.LaserScan ls) {
        LaserScan laserScan = new LaserScan();
        laserScan.setAngleIncrement(ls.getAngleIncrement());
        laserScan.setAngleMin(ls.getAngleMin());
        laserScan.setAngleMax(ls.getAngleMax());
        laserScan.setRangeMax(ls.getRangeMax());
        laserScan.setRangeMin(ls.getRangeMin());
        laserScan.setTimestamp((long) ls.getScanTime());
        double angle = ls.getAngleMin();
        if (ls.getRanges() != null) {
            List<Double> ranges = new ArrayList<Double>();
            float[] floats = ls.getRanges();
            for (float r : floats) {
                if (angle < Math.toRadians(23) && angle > Math.toRadians(-23)) {
                    if (Float.isNaN(r)) {
                        ranges.add(0.0);
                    } else {
                        ranges.add((double) r);
                    }

                }
                angle += ls.getAngleIncrement();
            }
            laserScan.setRanges(ranges);
        }
        return laserScan;
    }

    public static double getYaw(double x, double y, double z, double w) {
        return Math.atan2(2.0 * (x * y + z * w), w * w + x * x - y * y - z * z);
    }

    public static void connectToRos(AbstractNodeMain node) {
//            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.93.220:11311"));
//            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.95.50:11311"));
//            nodeConfiguration = NodeConfiguration.newPublic("192.168.1.6", new URI("http://192.168.1.6:11311"));
        connectToRos(node, "156.56.93.59", "156.56.93.220");
//        connectToRos(node, "156.56.93.59", "149.160.240.165");
    }

    public static void connectToRos(AbstractNodeMain node, String host, String rosHost) {
        NodeConfiguration nodeConfiguration;
        try {
            nodeConfiguration = NodeConfiguration.newPublic(host, new URI("http://" + rosHost + ":11311"));
            NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
            nodeMainExecutor.execute(node, nodeConfiguration);
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
    }

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

    public static double quantarianToRad(Quaternion q) {
//        return new Matrix3(q).getEulerYPR().yaw;
        return getYaw(q.getX(), q.getY(), q.getZ(), q.getW());
    }
}
