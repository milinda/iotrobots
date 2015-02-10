package cgl.iotrobots.sim;

import cgl.iotrobots.slam.core.app.GFSAlgorithm;
import cgl.iotrobots.slam.core.app.LaserScan;
import cgl.iotrobots.slam.core.gridfastsalm.GridSlamProcessor;
import cgl.iotrobots.slam.core.utils.DoubleOrientedPoint;
import cgl.iotrobots.slam.threading.ParallelGridSlamProcessor;
import cgl.iotrobots.slam.utils.*;
import nav_msgs.Odometry;
import org.apache.commons.lang3.tuple.Pair;

import java.io.BufferedReader;
import java.util.concurrent.BlockingQueue;

public class TurtleSimulator {
    public static final int SENSORS = 360;

    public static final double ANGLE = 2 * Math.PI;

    GFSAlgorithm gfsAlgorithm = new GFSAlgorithm();

    BufferedReader br = null;

    int parallel = 2;

    MapUI mapUI;

    RosMapPublisher rosMapPublisher = new RosMapPublisher();

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

        RosTurtle rosTurtle = new RosTurtle();
        TurtleUtils.connectToRos(rosTurtle);
        TurtleUtils.connectToRos(rosMapPublisher);

        Thread t = new Thread(new TurtleSimulator.Worker(rosTurtle.getQueue()));
        t.start();
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

    private class Worker implements Runnable {
        private BlockingQueue<Pair<Odometry, sensor_msgs.LaserScan>> queue;

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
                        gfsAlgorithm.laserScan(scan);

                        mapUI.setMap(gfsAlgorithm.getMap());
                        rosMapPublisher.addMap(gfsAlgorithm.getMap());
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }


}
