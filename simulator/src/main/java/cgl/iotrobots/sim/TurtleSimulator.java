package cgl.iotrobots.sim;

import cgl.iotcloud.core.transport.TransportConstants;
import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.GFSMap;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.streaming.Utils;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import cgl.iotrobots.slam.utils.*;
import cgl.iotrobots.utils.rabbitmq.*;
import com.esotericsoftware.kryo.Kryo;
import nav_msgs.Odometry;
import org.apache.commons.lang3.tuple.Pair;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;

public class TurtleSimulator {
    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();
    int parallel = 2;
    MapUI mapUI;
    RosMapPublisher rosMapPublisher = new RosMapPublisher();
    Kryo kryo = new Kryo();
    boolean storm = false;

    RabbitMQSender sender;
    RabbitMQReceiver receiver;
    RabbitMQReceiver bestReceiver;
    RabbitMQSender controlSender;

    public TurtleSimulator() {
        this(null, false);
    }

    public TurtleSimulator(String url, boolean storm) {
        mapUI = new MapUI();
        this.storm = storm;
        if (this.storm) {
            controlSender = new RabbitMQSender(url, "simbard_control");
            sender = new RabbitMQSender(url, "simbard_laser");
            try {
                receiver = new RabbitMQReceiver(url, "simbard_map");
                bestReceiver = new RabbitMQReceiver(url, "simbard_best");
                sender.open();
                controlSender.open();
                receiver.listen(new MapReceiver());
                bestReceiver.listen(new BestParticleReceiver());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void start(boolean parallel) throws InterruptedException {
        if (!parallel) {
            gfsAlgorithm.gsp = new GridSlamProcessor();
        } else {
            gfsAlgorithm.gsp = new ParallelGridSlamProcessor();
        }
        gfsAlgorithm.init();

        RosTurtle rosTurtle = new RosTurtle();
        TurtleUtils.connectToRos(rosTurtle);
        TurtleUtils.connectToRos(rosMapPublisher);

        Thread t = new Thread(new TurtleSimulator.Worker(rosTurtle.getQueue()));
        t.start();
    }

    public static void main(String[] args) throws InterruptedException {
        TurtleSimulator simulator = new TurtleSimulator("amqp://localhost:5672", true);
        if (args.length > 0) {
            simulator.start(true);
            simulator.parallel = Integer.parseInt(args[0]);
        } else {
            simulator.start(false);
        }
    }

    private class Worker implements Runnable {
        private BlockingQueue<Pair<Odometry, sensor_msgs.LaserScan>> queue;

        long lastTime = System.currentTimeMillis();

        private Worker(BlockingQueue<Pair<Odometry, sensor_msgs.LaserScan>> queue) {
            this.queue = queue;
        }

        @Override
        public void run() {
            while (true) {
                try {
                    Pair<Odometry, sensor_msgs.LaserScan> pair = null;
                    while (queue.size() > 0) {
                        pair = queue.take();
                    }
                    if (pair != null) {
                        LaserScan scan = TurtleUtils.turtleScanToLaserScan(pair.getRight());
                        Odometry odometry = pair.getLeft();
                        double rad = TurtleUtils.quantarianToRad(odometry.getPose().getPose().getOrientation());
                        System.out.format("received odometry: x = %f, y = %f, theta = %f\n", odometry.getPose().getPose().getPosition().getX(), odometry.getPose().getPose().getPosition().getY(), rad);
                        DoubleOrientedPoint lastPose = new DoubleOrientedPoint(odometry.getPose().getPose().getPosition().getX(),
                                odometry.getPose().getPose().getPosition().getY(), rad);
                        scan.setPose(lastPose);

                        if (!storm) {
                            gfsAlgorithm.laserScan(scan);
                            mapUI.setMap(gfsAlgorithm.getMap());
                            rosMapPublisher.addMap(gfsAlgorithm.getMap());
                        } else {
                            sendToStorm(scan);
                        }
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

        public void sendToStorm(LaserScan laserScan) {
            byte []body = Utils.serialize(kryo, laserScan);
            Map<String, Object> props = new HashMap<String, Object>();
            props.put("time", System.currentTimeMillis());
            props.put(TransportConstants.SENSOR_ID, System.currentTimeMillis());

            if (System.currentTimeMillis() - lastTime > 500) {
                lastTime = System.currentTimeMillis();
                Message message = new Message(body, props);
                try {
                    sender.send(message, "test.test.laser_scan");
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }



    private class BestParticleReceiver implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.best");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.best");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            Object time = message.getProperties().get("time");
            Long t = Long.parseLong(time.toString());
        }
    }

    private class MapReceiver implements MessageHandler {
        @Override
        public Map<String, String> getProperties() {
            Map<String, String> props = new HashMap<String, String>();
            props.put(MessagingConstants.RABBIT_ROUTING_KEY, "test.test.map");
            props.put(MessagingConstants.RABBIT_QUEUE, "test.test.map");
            return props;
        }

        @Override
        public void onMessage(Message message) {
            GFSMap map = (GFSMap) Utils.deSerialize(kryo, message.getBody(), GFSMap.class);
            Object time = message.getProperties().get("time");
            Long t = Long.parseLong(time.toString());
            System.out.println("*******************Map Time: " + (System.currentTimeMillis() - t));
            mapUI.setMap(map);
            rosMapPublisher.addMap(map);
        }
    }
}
