package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import geometry_msgs.Quaternion;
import nav_msgs.Odometry;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Subscriber;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

public class TurtleSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    BufferedReader br = null;

    int parallel = 2;

    MapUI mapUI;

    public TurtleSimulator() {
        mapUI = new MapUI();
    }

    public void start(boolean parallel) throws InterruptedException {
        // nothing particular in this case
        if (!parallel) {
            gfsAlgorithm.gsp_ = new GridSlamProcessor();
        } else {
            gfsAlgorithm.gsp_ = new ParallelGridSlamProcessor();
        }
        gfsAlgorithm.init();


        connectToRos();

        Thread t = new Thread(new Worker());
        t.start();
    }

    private void connectToRos() {
        // register with ros_java
        NodeConfiguration nodeConfiguration;
        try {
            nodeConfiguration = NodeConfiguration.newPublic("156.56.93.59", new URI("http://156.56.93.220:11311"));
            NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
            RosTurtle turtle = new RosTurtle();
            nodeMainExecutor.execute(turtle, nodeConfiguration);
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) throws InterruptedException {
        TurtleSimulator simulator = new TurtleSimulator();
        if (args.length > 0) {
            simulator.start(true);
            simulator.parallel = Integer.parseInt(args[0]);
        } else {
            simulator.start(false);
        }
    }

    public class RosTurtle extends AbstractNodeMain {
        private String name = "/ts_controller";

        public RosTurtle() {
        }

        public GraphName getDefaultNodeName() {
            return GraphName.of("/" + name);
        }

        @Override
        public void onStart(final ConnectedNode connectedNode) {
            System.out.println("Starting....");
            final Subscriber<Odometry> odometrySubscriber =
                    connectedNode.newSubscriber("/odom", Odometry._TYPE);

            final Subscriber<sensor_msgs.LaserScan> laserScanSubscriber =
                    connectedNode.newSubscriber("/scan", sensor_msgs.LaserScan._TYPE);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException ie) {
                ie.printStackTrace();
            }

            odometrySubscriber.addMessageListener(new MessageListener<Odometry>() {
                @Override
                public void onNewMessage(Odometry odometry) {
                    double rad = quantarianToRad(odometry.getPose().getPose().getOrientation());
                    if (state == State.WAITING_LASER_SCAN) {
                        lastPose = new DoubleOrientedPoint(odometry.getPose().getPose().getPosition().getX(),
                                odometry.getPose().getPose().getPosition().getY(), rad);
                    }
                }
            });

            laserScanSubscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
                @Override
                public void onNewMessage(sensor_msgs.LaserScan laserScanMsg) {
//                    LaserScan scan = turtleScanToLaserScan(laserScanMsg);
                    // update this with odomentry
//                    if (lastPose != null) {
//                        scan.setPose(lastPose);
//                        gfsAlgorithm.laserScan(scan);
//                        mapUI.setMap(gfsAlgorithm.getMap());
//                    }

                    if (state == State.WAITING_LASER_SCAN) {
                        state = State.UPDATING_LASER_SCAN;
                        scan = turtleScanToLaserScan(laserScanMsg);
                        state = State.WAITING_COMPUTING;
                    }
                }
            });

            // This CancellableLoop will be canceled automatically when the node shuts down.
            connectedNode.executeCancellableLoop(new CancellableLoop() {
                @Override
                protected void loop() throws InterruptedException {

                }
            });
        }

        public void onShutdown(Node node) {
            node.shutdown();
        }
    }

    DoubleOrientedPoint lastPose;
    enum State {
        UPDATING_LASER_SCAN,
        WAITING_COMPUTING,
        COMPUTING,
        WAITING_LASER_SCAN,
    }
    LaserScan scan;
    State state = State.WAITING_LASER_SCAN;
    boolean init = false;

    private void init(LaserScan scanI) {
        gfsAlgorithm.initMapper(scanI);
    }

    private double quantarianToRad(Quaternion q) {
        return Math.atan2(2.0 * (q.getY() * q.getZ() + q.getW() * q.getX()), q.getW() * q.getW() - q.getX() * q.getX() - q.getY() * q.getY() + q.getZ() * q.getZ());
    }

    private class Worker implements Runnable {
        @Override
        public void run() {
            while (true) {
                if (state == State.WAITING_COMPUTING) {
                    state = State.COMPUTING;
                    System.out.println("Start computing......................");
                    if (scan != null && lastPose != null) {
                        if (!init) {
                            init(scan);
                            init = true;
                        } else {
                            scan.setPose(lastPose);
                            gfsAlgorithm.laserScan(scan);
                            mapUI.setMap(gfsAlgorithm.getMap());
                        }
                    }
                    state = State.WAITING_LASER_SCAN;
                    System.out.println("End computing......................");
                }
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }


    private LaserScan turtleScanToLaserScan(sensor_msgs.LaserScan ls) {
        LaserScan laserScan = new LaserScan();
        laserScan.setAngleIncrement(ls.getAngleIncrement());
        laserScan.setAngleMin(ls.getAngleMin());
        laserScan.setAngleMax(ls.getAngleMax());
        laserScan.setRangeMax(ls.getRangeMax());
        laserScan.setRangeMin(ls.getRangeMin());
        laserScan.setTimestamp((long) ls.getScanTime());
        if (ls.getRanges() != null) {
            List<Double> ranges = new ArrayList<Double>();
            float[] floats = ls.getRanges();
            for (float r : floats) {
                if (Float.isNaN(r)) {
                    ranges.add((double) ls.getRangeMax());
                }
                ranges.add((double) r);
            }
            laserScan.setRanges(ranges);
        }
        return laserScan;
    }

    private DoubleOrientedPoint getOdomPose(long t) {
        return null;
    }
}
